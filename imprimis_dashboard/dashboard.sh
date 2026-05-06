#!/bin/bash
# Usage: sudo bash install_dashboard.sh

set -e #failsafe

if [ "$EUID" -ne 0 ]; then #must be root user
  echo "Run with sudo: sudo bash install_dashboard.sh"
  exit 1
fi

echo "==> Updating apt"
apt update

echo "==> Installing foxglove_bridge and diagnostic_aggregator"
apt install -y \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-diagnostic-aggregator

echo "==> Installing Cockpit (newer version from PPA for plugin compatibility)" #ubuntu repo has outdated one
apt install -y software-properties-common
add-apt-repository -y ppa:cockpit-project/cockpit
apt update
apt install -y cockpit

echo "==> Adding Clearpath apt repo for ROS2 diagnostics plugin"
wget -qO - https://packages.clearpathrobotics.com/public.key | \
    gpg --dearmor -o /usr/share/keyrings/clearpath-archive-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/clearpath-archive-keyring.gpg] https://packages.clearpathrobotics.com/stable/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/clearpath-latest.list
apt update

echo "==> Installing cockpit-ros2-diagnostics plugin"
apt install -y cockpit-ros2-diagnostics

echo "==> Configuring Cockpit to allow HTTP (required for plugin WebSocket)"
mkdir -p /etc/cockpit
cat > /etc/cockpit/cockpit.conf << 'EOF'
[WebService]
AllowUnencrypted=true
EOF

echo "==> Starting Cockpit"
systemctl enable --now cockpit.socket
systemctl restart cockpit

echo
echo "============================================"
echo "Install complete."
echo
echo "Next steps:"
echo "  1. Start the dashboard services:"
echo "     ros2 launch ~/imprimis/imprimis_dashboard/launch/dashboard.launch.py"
echo "  2. Open http://<robot-hostname>:9090 in your browser"
echo "  3. Log in with your Linux username and password"
echo "  4. Click the 'ROS 2 diagnostics' tab in the sidebar"
echo "============================================"