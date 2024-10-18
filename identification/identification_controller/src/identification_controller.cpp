// Copyright 2022, ICube Laboratory, University of Strasbourg
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "identification_controller/identification_controller.hpp"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace identification_controller
{
using hardware_interface::LoanedCommandInterface;

IdentificationController::IdentificationController()
: controller_interface::ControllerInterface(),
  rt_command_ptr_(nullptr),
  joints_command_subscriber_(nullptr)
{
}

CallbackReturn IdentificationController::on_init()
{
  try
  {
    // definition of the parameters that need to be queried from the
    // controller configuration file with default values
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    // auto_declare<std::vector<double>>("stiffness", std::vector<double>());
    // auto_declare<std::vector<double>>("damping", std::vector<double>());
    // auto_declare<std::vector<double>>("mass", std::vector<double>());
    auto_declare<std::vector<double>>("max_torque", std::vector<double>());
    auto_declare<std::vector<double>>("delay_at_start",std::vector<double>());
    auto_declare<std::vector<double>>("inputs", std::vector<double>());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  point_id = 0;
  reset_fault_sent_ = false;
  return CallbackReturn::SUCCESS;
}

CallbackReturn IdentificationController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // getting the names of the joints to be controlled
  joint_names_ = get_node()->get_parameter("joints").as_string_array();

  if (joint_names_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
    return CallbackReturn::FAILURE;
  }
  // getting the impedance parameters
  // stiffness_ = get_node()->get_parameter("stiffness").as_double_array();
  // damping_ = get_node()->get_parameter("damping").as_double_array();
  // mass_ = get_node()->get_parameter("mass").as_double_array();
  max_torque_ = get_node()->get_parameter("max_torque").as_double_array();
  auto del = get_node()->get_parameter("delay_at_start").as_double_array();
  delay_at_start_ = (int) del[0];
  inputs_ = get_node()->get_parameter("inputs").as_double_array();
  RCLCPP_INFO(get_node()->get_logger(), "inputs is of length : %d",inputs_.size());

  // if(stiffness_.empty())
  //   stiffness_.resize(joint_names_.size(),50.0);
  // if(damping_.empty())
  //   damping_.resize(joint_names_.size(),10.0);
  // if(mass_.empty())
  //   mass_.resize(joint_names_.size(),0.0);

  // if((stiffness_.size() != damping_.size()) || (stiffness_.size() != mass_.size())){
  //   RCLCPP_ERROR(get_node()->get_logger(), "incoherent size of impedance parameters");
  //   return CallbackReturn::FAILURE;
  // }

  // for(auto i = 0ul; i < stiffness_.size();i++){
  //   if (stiffness_[i] < 0 || damping_[i] < 0 || mass_[i] < 0)
  //   {
  //     RCLCPP_ERROR(get_node()->get_logger(), "wrong impedance parameters");
  //     return CallbackReturn::FAILURE;
  //   }
  // }
  // the desired position of the proxy point are queried from the proxy topic
  // and passed to update via a rt pipe
  joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
    "~/proxy", rclcpp::SystemDefaultsQoS(),
    [this](const CmdType::SharedPtr msg) { rt_command_ptr_.writeFromNonRT(msg); });

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}
// As impedance control targets the effort interface, it can be directly defined here
// without the need of getting as parameter. The effort interface is then affected to
// all controlled joints.
controller_interface::InterfaceConfiguration
IdentificationController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(joint_names_.size());
  for (const auto & joint_name : joint_names_)
  {
    conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_EFFORT);
    conf.names.push_back(joint_name + "/" + "reset_fault");
  }
  return conf;
}
// Impedance control requires both velocity and position states. For this reason
// there can be directly defined here without the need of getting as parameters.
// The state interfaces are then deployed to all targeted joints.
controller_interface::InterfaceConfiguration
IdentificationController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(joint_names_.size() * 2);
  for (const auto & joint_name : joint_names_)
  {
      conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
      conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return conf;
}

// Fill ordered_interfaces with references to the matching interfaces
// in the same order as in joint_names
template <typename T>
bool get_ordered_interfaces(
  std::vector<T> & unordered_interfaces, const std::vector<std::string> & joint_names,
  const std::string & interface_type, std::vector<std::reference_wrapper<T>> & ordered_interfaces)
{
  for (const auto & joint_name : joint_names)
  {
    for (auto & command_interface : unordered_interfaces)
    {
      if (
        (command_interface.get_name() == joint_name +"/"+ interface_type) &&
        (command_interface.get_interface_name() == interface_type))
      {
        ordered_interfaces.push_back(std::ref(command_interface));
      }
    }
  }

  return joint_names.size() == ordered_interfaces.size();
}

CallbackReturn IdentificationController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::vector<std::reference_wrapper<LoanedCommandInterface>> ordered_interfaces;
  if (
    !get_ordered_interfaces(
      command_interfaces_, joint_names_, hardware_interface::HW_IF_EFFORT, ordered_interfaces)) //||
    // command_interfaces_.size() != ordered_interfaces.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu position command interfaces, got %zu", joint_names_.size(),
      ordered_interfaces.size());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  if(!reset_fault_sent_)
  {
      command_interfaces_[1].set_value(0);
      // std::this_thread::sleep_for(std::chrono::seconds(2));
      command_interfaces_[1].set_value(1);
      // reset_fault_sent_ = true;
  }
  return CallbackReturn::SUCCESS;
}
// When deactivating the controller, the effort command on all joints is set to 0
CallbackReturn IdentificationController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
    for (auto index = 0ul; index < joint_names_.size(); ++index)
    {
        command_interfaces_[index].set_value(0.0);
    }
  return CallbackReturn::SUCCESS;
}
// main control loop function getting the state interface and writing to the command interface
controller_interface::return_type IdentificationController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  counter_ ++;
  if(counter_ < delay_at_start_)  // delay at the beginning so that the motor driver starts working
    return controller_interface::return_type::OK;


  if(!reset_fault_sent_)
  {
      command_interfaces_[1].set_value(0);
      command_interfaces_[1].set_value(1);
      reset_fault_sent_ = true;
  }
  // getting the data from the subscriber using the rt pipe
  // auto proxy = rt_command_ptr_.readFromRT();
  // // no command received yet
  // if (!proxy || !(*proxy))
  // {
  //   return controller_interface::return_type::OK;
  // }
  // //checking proxy data validity
  // if ((*proxy)->joint_names.size() != joint_names_.size() ||
  //     (*proxy)->points[0].positions.size() != joint_names_.size())  {
  //   RCLCPP_ERROR_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(), 1000,"command size does not match number of interfaces");
  //   return controller_interface::return_type::ERROR;
  // }

  //Impedance control loop
  for (auto index = 0ul; index < joint_names_.size(); ++index)
  {
    // the stats are given in the same order as defines in state_interface_configuration
    // double q = state_interfaces_[2*index].get_value();
    // double qv = state_interfaces_[2*index+1].get_value();
    // double qd = (*proxy)->points[0].positions[index];
    // double qdv = 0;
    // if((*proxy)->points[0].velocities.size() == joint_names_.size())
    //   qdv = (*proxy)->points[0].velocities[index];
    // double qda = 0;
    // if((*proxy)->points[0].accelerations.size() == joint_names_.size())
    //   qda = (*proxy)->points[0].accelerations[index];
    // double tau = mass_[index]*qda + stiffness_[index]*(qd-q) + damping_[index]*(qdv-qv);
    double max_pos = 2000;
    double tau = 0;
    if(point_id < inputs_.size())
       tau = inputs_[point_id];
    
    if(tau > max_torque_[index]) 
    {
      tau = max_torque_[index];
    }
    else if (tau < -max_torque_[index])
    {
      tau = -max_torque_[index];
    }

//Test to make the robot moves by it self
    // if((state_interfaces_[0].get_value() > max_pos && state_interfaces_[1].get_value() > 0) || (state_interfaces_[0].get_value() < -max_pos && state_interfaces_[1].get_value() < 0))
    //   tau = -tau;
    
    // if( tau > 0)  //state_interfaces_[0].get_value() > 2 &&
    //     tau += 37;
    // if(tau < 0)
    //     tau -= 37;

    command_interfaces_[index].set_value(tau);
    point_id++;

    if(counter_%5000 == 0)
      RCLCPP_INFO( get_node()->get_logger(), " Torque command %d  = %f ", point_id, tau);
  }

  return controller_interface::return_type::OK;
}

}  // namespace identification_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  identification_controller::IdentificationController, controller_interface::ControllerInterface)
