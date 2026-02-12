#!/bin/bash

# Allow multicast
sudo ufw allow in proto udp to 224.0.0.0/4
sudo ufw allow in proto udp from 224.0.0.0/4

# Add imprimis meshes to gazebo's path
source ./install/setup.bash
LINE='export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:$(ros2 pkg prefix imprimis_description)/share'
grep -qxF "$LINE" "$HOME/.bashrc" || echo "$LINE" >>  "$HOME/.bashrc"
source ~/.bashrc




