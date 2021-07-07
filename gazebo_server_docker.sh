#!/bin/bash
set -e
function cleanup {
	docker stop gazebo && docker rm gazebo
}
trap cleanup EXIT
docker run -it -v="/tmp/.gazebo/:/root/.gazebo/" --name gazebo rb-rca5-gazebo:latest
