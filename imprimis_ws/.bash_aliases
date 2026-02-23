
# Check you .bashrc and confirm that this is there and the path matches.
# if [ -f ~/Desktop/imprimis/imprimis_ws/.bash_aliases ]; then
#    . ~/Desktop/imprimis/imprimis_ws/.bash_aliases
# fi

# All the aliases are here
alias launch_hardware='ros2 launch imprimis_hardware_platform imprimis_hardware.launch.py'
alias launch_localization='ros2 launch imprimis_navigation localization.launch.py'
alias launch_nav='ros2 launch imprimis_navigation basic_nav.launch.py'

goal_pub () {
  lat=${1:--22.98669}
  long=${2:--43.2025}
  ros2 topic pub --once /gps/goal sensor_msgs/msg/NavSatFix "{
  header: {frame_id: 'gps_link'},
  status: {status: 0, service: 1},
  latitude: $lat,
  longitude: $long, 
}"
}