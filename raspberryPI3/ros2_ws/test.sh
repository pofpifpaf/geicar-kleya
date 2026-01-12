#!/bin/bash

cd src/hmi/hmi/python_files/
uvicorn backend_api:app --host 0.0.0.0 --port 8000 &
ros2 launch geicar_start geicar.launch.py &
streamlit run hmi.py &
ros2 run imu_rx ecompass_node.py
