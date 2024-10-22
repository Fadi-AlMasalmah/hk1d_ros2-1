import rclpy
import numpy as np
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import random
from std_msgs.msg import Float64 as FloatMsg



class RefPosPublisher(Node):
    def __init__(self, joint_name, max_position, min_position, Ts, offset_position):
        super().__init__('signal_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, 'identification_controller_cl/ref_pos', 10)
        self.joint_name = joint_name
        self.max_position = max_position
        self.min_position = min_position
        self.Ts = Ts  # Sampling time (in seconds)
        self.offset_position = offset_position
        self.signal_sequence = []  # List to hold the entire signal sequence
        self.current_idx = 0  # To track which point to publish next

        # publish simulation time to syncronize ros2 bags for the  plots
        simulation_time_topic_name = 'simulation_time'
        self._publisher_simulation_time = self.create_publisher(
            FloatMsg,
            simulation_time_topic_name,
            5
        )
        self._t0 = self.get_clock().now()

        # Create a timer to publish the signal at every time step Ts
        self.timer = self.create_timer(Ts, self.timer_callback)



    def create_ramp_signal(self, duration, start_pos, end_pos, steps):
        return np.linspace(start_pos, end_pos, steps)

    def create_square_signal(self, duration, amplitude, frequency, steps):
        t = np.linspace(0, duration, steps)
        return np.sign(np.sin(2 * np.pi * frequency * t)) * amplitude

    def create_sine_signal(self, duration, amplitude, frequency, steps):
        t = np.linspace(0, duration, steps)
        return amplitude * np.sin(2 * np.pi * frequency * t)

    def generate_random_signal(self, signal_type, duration, frequency=None):
        amplitude = random.uniform(self.min_position, self.max_position)
        steps = int(duration / self.Ts)  # Number of steps based on duration and sampling time
        if frequency is None:
            frequency = random.uniform(0.1, 1.0)  # Random frequency if not provided
        if signal_type == 'ramp':
            return self.create_ramp_signal(duration, self.min_position, amplitude, steps)
        elif signal_type == 'square':
            return self.create_square_signal(duration, amplitude, frequency, steps)
        elif signal_type == 'sine':
            return self.create_sine_signal(duration, amplitude, frequency, steps)
        else:
            return np.zeros(steps)  # Default to zero if type not recognized

    def generate_signal_sequence(self, sequence):
        # Create the entire signal sequence at once by concatenating different signal types
        self.signal_sequence = []
        for signal_type, duration, frequency in sequence:
            signal = self.generate_random_signal(signal_type, duration, frequency)
            self.signal_sequence.extend(signal)

    def timer_callback(self):
        # Publish one point from the signal sequence at each timer callback
        if self.current_idx < len(self.signal_sequence):
            pos = self.signal_sequence[self.current_idx] + self.offset_position
            traj_msg = JointTrajectory()
            traj_msg.joint_names = [self.joint_name]

            point = JointTrajectoryPoint()
            point.positions = [pos]
            point.time_from_start = rclpy.duration.Duration(seconds=self.current_idx * self.Ts).to_msg()
            traj_msg.points = [point]

            self.publisher_.publish(traj_msg)
            self.current_idx += 1
        else:
            self.get_logger().info("Finished publishing the entire signal sequence.")
        # publish simulation time
        timestamp = self.get_clock().now().to_msg()
        simulation_time_msg = FloatMsg()
        simulation_time_msg.data = self.current_time
        self._publisher_simulation_time.publish(simulation_time_msg)

    def run(self, sequence):
        self.generate_signal_sequence(sequence)
        self.get_logger().info("Generated signal sequence. Now publishing...")

    @property
    def current_time(self):
        if (self._t0 is None):
            return 0.0
        else:
            current_t_ns = self.get_clock().now().nanoseconds
            return (current_t_ns - self._t0.nanoseconds)*(1e-9)


def main(args=None):
    rclpy.init(args=args)
    joint_name = 'joint_1'
    max_position = 0.4
    min_position = -0.4
    Ts = 0.002  # Sampling period of 0.1 seconds
    offset_position = 0.3 # Offset position for the signal (blue robot should be added 0.3 to avoid limits)

    # Create the publisher node
    signal_publisher = RefPosPublisher(joint_name, max_position, min_position, Ts, offset_position)

    # Define a sequence of signals: (signal_type, duration, frequency)
    signal_sequence = [
        ('square', 2, 1),     
        ('square', 0.5, 1),
        ('square', 0.5, 1),
        ('sine', 2, 3.0),        
        ('square', 0.5, 0.5),
        ('square', 0.5, 0.5),
        ('sine', 2, 2.0),      
        ('square', 0.5, 0.5),
        ('square', 0.5, 0.5),
        ('sine', 4, 5.0),
        ('square', 0.3, 20),    
        ('square', 2, 1),     
        ('sine', 1, 20.0), 
        ('square', 2, 2),     
        ('square', 0.4, 15),    
        ('sine', 2, 2.0),
        ('square', 2, 3),     
        ('square', 0.4, 30),    
        ('square', 2, 4),     
        ('sine', 1, 7.0),
        ('square', 0.4, 12),     
        ('square', 1, 6),     
        ('square', 1, 5),     
        ('square', 1, 7),     
        ('square', 0.4, 25),  
        ('square', 0.4, 17),  
        ('ramp', 1, None),    
        
        
    ]

    # Generate the signal sequence and start the publishing process
    signal_publisher.run(signal_sequence)

    # Keep the node spinning to allow the timer to publish signals
    rclpy.spin(signal_publisher)

    # Shutdown ROS node
    rclpy.shutdown()


if __name__ == '__main__':
    main()
