#!/bin/bash

# Get current script directory
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "$(readlink -f -- "$0")")" && pwd)"

# Allow multicast
sudo ufw allow in proto udp to 224.0.0.0/4
sudo ufw allow in proto udp from 224.0.0.0/4

# Source imprimis automatically (assumes this script file is in the workspace root)
LINE1="source $SCRIPT_DIR/install/setup.bash"

# Add imprimis meshes to gazebo's path
LINE2='export GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH:+$GZ_SIM_RESOURCE_PATH:}$(ros2 pkg prefix imprimis_description)/share' # harmonic (jazzy)
LINE3='export IGN_GAZEBO_RESOURCE_PATH=${IGN_GAZEBO_RESOURCE_PATH:+$IGN_GAZEBO_RESOURCE_PATH:}$(ros2 pkg prefix imprimis_description/share' # fortress (humble)

# Delete any modifications this script made in the previous run, if any
sed -i '/imprimis/d' ~/.bashrc

# Add new lines containing "imprimis" to bashrc
echo $LINE1 >> ~/.bashrc
echo $LINE2 >> ~/.bashrc
echo $LINE3 >> ~/.bashrc

# Run all those commands
source ~/.bashrc





