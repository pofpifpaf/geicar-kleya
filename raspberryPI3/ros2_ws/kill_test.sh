#!/bin/bash

pkill --signal 9 -f ros2
pkill --signal 9 -f uvicorn
pkill --signal 9 -f streamlit
pkill --signal 9 -f python3

ros2 daemon stop
ros2 daemon start
