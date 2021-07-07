#!/bin/bash
export GAZEBO_MASTER_IP=$(docker inspect --format '{{ .NetworkSettings.IPAddress }}' gazebo)
export GAZEBO_MASTER_URI=$GAZEBO_MASTER_IP:11345
./gazebo_client.sh
