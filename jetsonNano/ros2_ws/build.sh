rm -r build/ install/ log/
colcon build --packages-select interfaces
colcon build --packages-select usb_cam rplidar_ros2
colcon build --packages-select lane_detection