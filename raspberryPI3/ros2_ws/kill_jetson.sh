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
    pkill --signal 9 -f lane
    pkill --signal 9 -f cam
    pkill --signal 9 -f lidar
    pkill --signal 9 -f system
    echo "Done"
INNER
EOF

