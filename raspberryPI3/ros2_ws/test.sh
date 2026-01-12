#!/bin/bash

source install/setup.bash
cd src/hmi/hmi/python_files/

uvicorn backend_api:app --host 0.0.0.0 --port 8000 &
streamlit run hmi.py &

cd ../../../../

ros2 launch geicar_start geicar.launch.py &
ros2 run imu_rx ecompass_node --ros-args -p mag_offset_x:=-931.5 -p mag_offset_y:=87.0 -p mag_offset_z:=123.0 
