# A message tha contains the info to identify the HK1D robot parameters
std_msgs/Header header

# std_msgs/String joint_name
float64 measured_joint_velocity
float64 measured_joint_effort
float64 measured_joint_position
float64 measured_joint_acceleration
float64 reference_joint_position
float64 reference_effort
float64 external_effort