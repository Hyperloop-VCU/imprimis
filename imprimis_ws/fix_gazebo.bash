#!/bin/bash

# Get current script directory
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "$(readlink -f -- "$0")")" && pwd)"

# Allow multicast
sudo ufw allow in proto udp to 224.0.0.0/4
sudo ufw allow in proto udp from 224.0.0.0/4

# Source imprimis automatically and add imprimis meshes to gazebo's path
LINE1="source $SCRIPT_DIR/install/setup.bash"
LINE2='export GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH:+$GZ_SIM_RESOURCE_PATH:}$(ros2 pkg prefix imprimis_description)/share'

# Add those two commands to bashrc, if they aren't already in there
grep -qxF "$LINE1" "$HOME/.bashrc" || echo "$LINE1" >> "$HOME/.bashrc"
grep -qxF "$LINE2" "$HOME/.bashrc" || echo "$LINE2" >> "$HOME/.bashrc"

# Run those two commands
source ~/.bashrc





