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

#include "identification_controller_cl/identification_controller_cl.hpp"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace identification_controller_cl
{
using hardware_interface::LoanedCommandInterface;

IdentificationControllerCL::IdentificationControllerCL()
: controller_interface::ControllerInterface(),
  rt_command_ptr_(nullptr),
  joints_command_subscriber_(nullptr)
{
}

CallbackReturn IdentificationControllerCL::on_init()
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
    auto_declare<std::vector<double>>("ref_poses", std::vector<double>());
    auto_declare<std::vector<double>>("Kp", std::vector<double>());
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

CallbackReturn IdentificationControllerCL::on_configure(
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
  Kp_ = get_node()->get_parameter("Kp").as_double_array();
  Kd_ = get_node()->get_parameter("Kd").as_double_array();
  auto del = get_node()->get_parameter("dela_at_start_in_counts").as_double_array();
  delay_at_start_ = (int) del[0];
  max_pos_ = get_node()->get_parameter("max_pos").as_double_array();
  min_pos_ = get_node()->get_parameter("min_pos").as_double_array();

  // ref_poses_ = get_node()->get_parameter("ref_poses").as_double_array();
  // RCLCPP_INFO(get_node()->get_logger(), "ref_poses is of length : %d",ref_poses_.size());

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
  // the desired position of the ref_pos point are queried from the ref_pos topic
  // and passed to update via a rt pipe
  joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
    "~/ref_pos", rclcpp::SystemDefaultsQoS(),
    [this](const CmdType::SharedPtr msg) { rt_command_ptr_.writeFromNonRT(msg); });

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}
// As impedance control targets the effort interface, it can be directly defined here
// without the need of getting as parameter. The effort interface is then affected to
// all controlled joints.
controller_interface::InterfaceConfiguration
IdentificationControllerCL::command_interface_configuration() const
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
IdentificationControllerCL::state_interface_configuration() const
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

CallbackReturn IdentificationControllerCL::on_activate(
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
CallbackReturn IdentificationControllerCL::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
    for (auto index = 0ul; index < joint_names_.size(); ++index)
    {
        command_interfaces_[index].set_value(0.0);
    }
  return CallbackReturn::SUCCESS;
}
// main control loop function getting the state interface and writing to the command interface
controller_interface::return_type IdentificationControllerCL::update(const rclcpp::Time & time, const rclcpp::Duration & period)
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
  auto ref_pos_msg = rt_command_ptr_.readFromRT();
  // no command received yet
  if (!ref_pos_msg || !(*ref_pos_msg))
  {
    return controller_interface::return_type::OK;
  }
  //checking ref_pos data validity
  // ref_pos = (*ref_pos_msg)->data;
  ref_pos = (*ref_pos_msg)->points[0].positions[0];
  if (ref_pos < min_pos_[0]) 
  {
    ref_pos = min_pos_[0];
  }
  else if (ref_pos > max_pos_[0])
  {
    ref_pos = max_pos_[0];
  }

  //Impedance control loop
  for (auto index = 0ul; index < joint_names_.size(); ++index)
  {
    double tau = 0;
    double x_d = ref_pos;
    // if(point_id < ref_poses_.size())
    //    x_d = ref_poses_[point_id];
  
    tau = Kp_[0] * (x_d - state_interfaces_[0].get_value()) + Kd_[0] * (0 - state_interfaces_[1].get_value()) ;
    
    

//Test to make the robot moves by it self
    // if((state_interfaces_[0].get_value() > max_pos && state_interfaces_[1].get_value() > 0) || (state_interfaces_[0].get_value() < -max_pos && state_interfaces_[1].get_value() < 0))
    //   tau = -tau;    
    // if( tau > 0)  //state_interfaces_[0].get_value() > 2 &&
    //     tau += 37;
    // if(tau < 0)
    //     tau -= 37;

    if(tau > max_torque_[index]) 
      tau = max_torque_[index];
    else if (tau < -max_torque_[index])
      tau = -max_torque_[index];
    
    command_interfaces_[index].set_value(tau);
    point_id++;

    if(counter_%1000 == 0)
      RCLCPP_INFO( get_node()->get_logger(), " Torque command %d  = %f, and xd = %f ", point_id, tau, x_d);
  }

  return controller_interface::return_type::OK;
}

}  // namespace identification_controller_cl

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  identification_controller_cl::IdentificationControllerCL, controller_interface::ControllerInterface)

