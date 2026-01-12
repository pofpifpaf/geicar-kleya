source install/setup.bash
ros2 run imu_rx imu_rx_node &
ros2 run imu_rx ecompass_node &
ros2 topic echo /ecompass
