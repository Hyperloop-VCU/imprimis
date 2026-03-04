
# Check your .bashrc and confirm that this is there and the path matches.
# if [ -f ~/Desktop/imprimis/imprimis_ws/.bash_aliases ]; then
#    . ~/Desktop/imprimis/imprimis_ws/.bash_aliases
# fi

# If you change the location of this workspace on your computer, you need to change the path in .bashrc as well.


IMPRIMIS_ALIAS_BASH_SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)

alias imp='clear && cd $IMPRIMIS_ALIAS_BASH_SCRIPT_DIR'
alias imp-code='imp && code src'
alias launch_hardware='imp && ros2 launch imprimis_hardware_platform imprimis_hardware.launch.py'
alias launch_localization='imp && ros2 launch imprimis_navigation localization.launch.py'
alias launch_nav='imp && ros2 launch imprimis_navigation basic_nav.launch.py'
alias clear-build='imp && rm -rf build install log'
alias rebuild='imp && colcon build && source install/setup.bash'
alias rebuild-safe='imp && colcon build --executor sequential && source install/setup.bash'

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
