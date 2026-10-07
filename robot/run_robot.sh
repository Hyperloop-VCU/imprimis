#!/usr/bin/env bash
# run_robot.sh : one-click start of the IMPRIMIS lap software on the ROVER'S OWN COMPUTER (Ubuntu, ROS 2 Jazzy).
#
#   UNTESTED ON THE ROVER. It was written and tried on a computer with no robot attached. It starts
#   robot_mission.launch.py, which is itself untested on hardware. Treat the first use as a test:
#   wheels off the ground, motors off, then motors on with a hand on the red button.
#
# What it does, in order
#   1. finds the workspace beside this folder (../imprimis_ws) and ROS 2 Jazzy
#   2. reads the settings in robot.conf (made from robot.conf.example on the first run)
#   3. checks that the devices are plugged in, and says which are missing
#   4. builds the workspace if it has never been built or a source file is newer than the build
#   5. shows the safety reminder and waits for Enter
#   6. starts the software and keeps a log in ~/imprimis_logs
#
# Use
#   double-click the desktop icon made by install_desktop_launcher.sh, or:   bash run_robot.sh
#   bash run_robot.sh --check     everything except starting the software (steps 1 to 4)
#   bash run_robot.sh --yes       do not wait for Enter (for a test bench only)
# No "set -u" in this script: the ROS setup files read variables that are not set.
HERE="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
WS="$(cd "$HERE/.." && pwd)/imprimis_ws"
CONF="$HERE/robot.conf"
CHECK_ONLY=0
ASSUME_YES=0
PROBLEMS=0
for arg in "$@"; do
  case "$arg" in
    --check) CHECK_ONLY=1 ;;
    --yes) ASSUME_YES=1 ;;
    *) echo "unknown option: $arg   (options: --check, --yes)"; exit 2 ;;
  esac
done

finish() {      # keep the window open when started by double-click, so the last messages can be read
  echo
  if [ "$ASSUME_YES" = "0" ] && [ -t 0 ]; then read -r -p "Press Enter to close this window. " _; fi
  exit "${1:-0}"
}
say()  { printf '%s\n' "$*"; }
good() { printf '   [ ok ]      %s\n' "$*"; }
bad()  { printf '   [ MISSING ] %s\n' "$*"; PROBLEMS=$((PROBLEMS + 1)); }
note() { printf '   [ note ]    %s\n' "$*"; }

say
say " IMPRIMIS rover - lap software"
say " ============================="

# ---------------------------------------------------------------- 1. ROS and the workspace
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  say " ROS 2 Jazzy is not installed on this computer (/opt/ros/jazzy/setup.bash not found)."
  finish 1
fi
if [ ! -d "$WS/src/imprimis_mission" ]; then
  say " The workspace was not found at $WS"
  say " This folder (robot) must stay beside the folder imprimis_ws."
  finish 1
fi

# ---------------------------------------------------------------- 2. settings
if [ ! -f "$CONF" ]; then
  cp "$HERE/robot.conf.example" "$CONF"
  say " First run: the settings file was made at"
  say "   $CONF"
  say " Open it in a text editor and set at least COURSE_FILE and IMU_YAW_IN_BASE."
fi
# defaults, then the file
COURSE_FILE=""; MEMORY_DIR="$HOME/imprimis_lap_memory_robot"; TOP_SPEED="0.5"; IMU_YAW_IN_BASE="0.0"
USE_CAMERA="true"; CAMERA_SERIAL="_923322073287"; COLOR_FOV="1.204"; CONTROL_WINDOW="true"; UI_TYPE="none"
NAV2_PARAMS="Course2027"; ARMING_CHECK="true"; BOUNDARY_CHECK="true"; LIDAR_IP="192.168.100.201"
BOARD_A_PORT="/dev/ttyUSB0"; IMU_PORT="/dev/ttyUSB1"; GPS_PORT="/dev/ttyACM0"
# shellcheck disable=SC1090
source "$CONF"
COURSE_FILE="${COURSE_FILE/#\~/$HOME}"
MEMORY_DIR="${MEMORY_DIR/#\~/$HOME}"

say
say " Settings (from robot.conf)"
say "   course file      ${COURSE_FILE:-none}"
say "   lap memory       $MEMORY_DIR"
say "   top speed        $TOP_SPEED m/s"
say "   IMU turned by    $IMU_YAW_IN_BASE rad"
say "   camera           $USE_CAMERA   control window  $CONTROL_WINDOW"
say "   arming check     $ARMING_CHECK   GPS boundary    $BOUNDARY_CHECK"

# ---------------------------------------------------------------- 3. what is plugged in
say
say " Devices"
[ -e "$BOARD_A_PORT" ] && good "Board A on $BOARD_A_PORT" || bad "Board A on $BOARD_A_PORT (the motors cannot be driven without it)"
[ -e "$IMU_PORT" ] && good "IMU on $IMU_PORT" || bad "IMU on $IMU_PORT"
[ -e "$GPS_PORT" ] && good "GPS on $GPS_PORT" || bad "GPS on $GPS_PORT (the arming check will refuse automatic runs)"
if ping -c 1 -W 1 "$LIDAR_IP" > /dev/null 2>&1; then good "LiDAR answers at $LIDAR_IP"; else bad "LiDAR at $LIDAR_IP (no answer; check its power and the network cable)"; fi
if [ "$USE_CAMERA" = "true" ]; then
  if command -v lsusb > /dev/null 2>&1 && lsusb 2>/dev/null | grep -qi "8086:0b"; then good "an Intel RealSense camera on USB"; else bad "RealSense camera on USB (or lsusb is not installed)"; fi
fi
if id -nG | tr ' ' '\n' | grep -qx dialout; then good "this user may open serial ports (group dialout)"; else note "this user is not in the group dialout; serial ports may refuse to open (sudo usermod -aG dialout $USER, then log in again)"; fi
if [ -n "$COURSE_FILE" ]; then
  if [ -f "$COURSE_FILE" ]; then
    good "course file found"
    if grep -q '"datum_lat": 0.0' "$COURSE_FILE"; then note "the course file still has the template GPS datum (0.0); every automatic run will be refused until it is surveyed"; fi
  else
    bad "course file $COURSE_FILE"
  fi
else
  note "no course file set: mode switching only, no laps and no automatic runs"
fi
if [ "$IMU_YAW_IN_BASE" = "0.0" ]; then note "IMU_YAW_IN_BASE is still 0.0, the placeholder. Measure how the IMU is turned before trusting ramps and leveling."; fi

# ---------------------------------------------------------------- 4. build if needed
source /opt/ros/jazzy/setup.bash
cd "$WS" || finish 1
NEED_BUILD=0
if [ ! -f install/setup.bash ]; then
  NEED_BUILD=1
elif [ -n "$(find src \( -name '*.py' -o -name '*.cpp' -o -name '*.hpp' -o -name '*.h' -o -name 'package.xml' -o -name 'CMakeLists.txt' -o -name 'setup.py' \) -newer install/setup.bash -print -quit 2>/dev/null)" ]; then
  NEED_BUILD=1
fi
say
if [ "$NEED_BUILD" = "1" ]; then
  say " Building the workspace (the first time this takes several minutes)..."
  if ! colcon build; then
    say " THE BUILD FAILED. Read the messages above. Nothing was started."
    finish 1
  fi
  touch install/setup.bash
else
  say " Workspace is built and up to date."
fi
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

if [ "$CHECK_ONLY" = "1" ]; then
  say
  say " Check only: nothing was started. Items marked MISSING: $PROBLEMS."
  finish 0
fi

# ---------------------------------------------------------------- 5. the safety reminder
say
say " BEFORE YOU GO ON"
say "   1. The hand controller is ON. It must be on before the motors get power."
say "   2. Its switch decides. In MANUAL the robot ignores this software."
say "      Only in AUTONOMOUS do the commands of this software reach the motors."
say "   3. The red button and the keychain OFF stop the motors whatever this software does."
say "      Somebody has a hand near one of them."
say "   4. Top speed is $TOP_SPEED m/s. This launcher and its launch file are untested on the rover."
if [ "$PROBLEMS" -gt 0 ]; then
  say
  say "   $PROBLEMS item(s) above are MISSING. The software will start, but parts of it will not work."
fi
say
if [ "$ASSUME_YES" = "0" ]; then
  if ! read -r -p " Press Enter to start, or Ctrl+C to stop here. " _; then say; say " No keyboard: nothing was started."; exit 1; fi
fi

# ---------------------------------------------------------------- 6. start
mkdir -p "$MEMORY_DIR" "$HOME/imprimis_logs"
LOG="$HOME/imprimis_logs/robot_$(date +%Y-%m-%d_%H%M%S).log"
say
say " Starting. To stop the software, click this window and press Ctrl+C."
say " Log: $LOG"
say
ARGS=(memory_dir:="$MEMORY_DIR" top_speed:="$TOP_SPEED" imu_yaw_in_base:="$IMU_YAW_IN_BASE" use_camera:="$USE_CAMERA"
      camera_serial:="$CAMERA_SERIAL" color_fov:="$COLOR_FOV" gui:="$CONTROL_WINDOW" ui_type:="$UI_TYPE" nav2_params:="$NAV2_PARAMS"
      arming_check:="$ARMING_CHECK" boundary_check:="$BOUNDARY_CHECK")
if [ -n "$COURSE_FILE" ] && [ -f "$COURSE_FILE" ]; then ARGS+=(course_file:="$COURSE_FILE"); fi
ros2 launch imprimis_mission robot_mission.launch.py "${ARGS[@]}" 2>&1 | tee "$LOG"
say
say " The software has stopped."
finish 0
