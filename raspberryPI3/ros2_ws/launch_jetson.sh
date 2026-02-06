#!/bin/bash

HOST="jetson@192.168.1.174"
CONTAINER="ros-humble"

ssh "$HOST" << 'EOF'
  set -e

  echo "Connected to host"

  # Start container if not running
  if ! docker inspect -f '{{.State.Running}}' ros-humble 2>/dev/null | grep -q true; then
    echo "Starting container ros-humble"
    docker start ros-humble
  fi

  # Now exec inside it
  docker exec -i ros-humble bash << 'INNER'
    echo "Inside container"
    cd /root/test/feature-4-LCA/geicar-kleya/jetsonNano/ros2_ws/
    source install/setup.bash
    ros2 launch geicar_start_jetson geicar.jetson.launch.py &
    echo "Done"
INNER
EOF

