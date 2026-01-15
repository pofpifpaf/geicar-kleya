rm -r install/adas build/adas
rm -r install/imu_rx build/imu_rx
rm -r install/hmi build/hmi
rm -r install/geicar_start build/geicar_start
colcon build --packages-select adas imu_rx hmi
colcon build --packages-select geicar_start
