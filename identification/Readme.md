
# Identification Of hk1d Robot

This package is developed under ROS2 Humble.

## Usage

1. Start the EtherCat master

```bash
sudo /etc/init.d/ethercat start
```

To check if what EtherCat slaves are connected:

```bash
ethercat slaves
```


2. Choose the identification method in the yaml file (using ref-pos topic, or to identify from external force):

```bash
src/hk1d_ros2/identification/identification_bringup/config/hk1d_ident_controllers.yaml
```

3. Launch the robot with the identification controller

```bash 
ros2 launch identification_bringup hk1d_ident.launch.py use_fake_hardware:=false
```

To choose which robot to launch, you can pass the parameters [sensor_ec_id, motor_ec_id]. For example, for the blue robot:

```bash 
ros2 launch identification_bringup hk1d_ident.launch.py use_fake_hardware:=false motor_ec_id:=0 sensor_ec_id:=1
```

For the red robot:
```bash 
ros2 launch identification_bringup hk1d_ident.launch.py use_fake_hardware:=false motor_ec_id:=2 sensor_ec_id:=3
```

4. you can use plotjuggler config file to visualize in plotjuggler.

5. In another terminal, navigate to the data folder and record with ros2bag:

```bash
ros2 bag record --storage mcap /identification_controller_cl/identification_info  /simulation_time
```

6. If ref-pos identification mode is chosen, in another terminal, launch the ref pos publisher  (remeber to source):

```bash
ros2 run identification_node ref_pos_publisher 
```

If the identification mode is force_sensor, start moving the robot. At the end, stop the recording. Take the file and use the hk1d_identification_notebooks/rosbags_to_mat.ipynb file to convert it to .mat. And then use GitHub: hk1d_identification_MATLAB (Fadi Al-masalmah) to identify the model.

