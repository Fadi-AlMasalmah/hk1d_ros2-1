# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

# from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from math import sin
from std_msgs.msg import Float64 as FloatMsg


class IdentificationNode(Node):

    def __init__(self):
        super().__init__('identification_node')
        self.publisher_ = self.create_publisher(JointTrajectory, 'identification_controller_cl/ref_pos', 10)
        timer_period = 0.002  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        # publish simulation time to syncronize ros2 bags for the  plots
        simulation_time_topic_name = 'simulation_time'
        self._publisher_simulation_time = self.create_publisher(
            FloatMsg,
            simulation_time_topic_name,
            5
        )
        self._t0 = self.get_clock().now()


    def timer_callback(self):
        msg = JointTrajectory()
        msg.joint_names = ["joint_1"]

        point = JointTrajectoryPoint()
        point.positions = [0.2* sin(self.i/50)] 
        point.time_from_start.sec = 0
        # point.time_from_start.nanosec = 200000000
        msg.points = [point]
        msg.header.stamp = self.get_clock().now().to_msg()

        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

        # publish simulation time
        timestamp = self.get_clock().now().to_msg()
        simulation_time_msg = FloatMsg()
        simulation_time_msg.data = self.current_time
        self._publisher_simulation_time.publish(simulation_time_msg)

    @property
    def current_time(self):
        if (self._t0 is None):
            return 0.0
        else:
            current_t_ns = self.get_clock().now().nanoseconds
            return (current_t_ns - self._t0.nanoseconds)*(1e-9)


def main(args=None):
    rclpy.init(args=args)

    identification_node = IdentificationNode()

    rclpy.spin(identification_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    identification_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
