# One-Click Start for the Rover

Written October 4, 2026. UNTESTED ON THE ROVER. It was tried on a Linux computer with no robot attached.

## What this folder is

The rover's computer runs Linux, so a Windows batch file does not work on it. This folder holds the Linux equivalent: a script, and an installer that puts a double-click icon for it on the desktop.

- `run_robot.sh` is the launcher. It checks the devices, builds the workspace if needed, shows the safety reminder, waits for Enter, and starts `robot_mission.launch.py`.
- `install_desktop_launcher.sh` is run once. It puts two icons on the desktop: Run IMPRIMIS Rover, and Check IMPRIMIS Rover.
- `robot.conf.example` holds the settings with their explanations. The first run copies it to `robot.conf`, which is the file to edit.

## Getting it onto the rover

This copy of the repository is not the one on the team's GitHub. It has the lap software, the arming check and this folder in it. Copy the whole `imprimis` folder to the rover's computer, for example on a USB stick.

Do not copy it over the team's own copy at `~/Desktop/imprimis`. Put it beside it under another name, for example `~/imprimis_laps`. The two can live side by side, and the team's copy keeps working as before.

## First time on the rover

1. Open a terminal in this folder and run `bash install_desktop_launcher.sh`.
2. Double-click Check IMPRIMIS Rover. The first run makes `robot.conf`, builds the workspace, and lists which devices it can see. It starts nothing.
3. Open `robot.conf` in a text editor. Set `COURSE_FILE` and `IMU_YAW_IN_BASE`. Leave `TOP_SPEED` at 0.5.
4. Double-click Check IMPRIMIS Rover again until nothing is marked MISSING.

## Every time after that

1. Follow the team's turn-on procedure. The hand controller is on before the motors.
2. Double-click Run IMPRIMIS Rover.
3. Read the device list and the reminder. Press Enter.
4. To stop the software, click the window and press Ctrl+C. To stop the robot, use the red button or the keychain OFF. Do not rely on Ctrl+C to stop the robot.

Each run writes a log to `~/imprimis_logs`.

## What to expect the first time

The launcher and the launch file it starts have never run on the rover. Expect to fix things. Do the first start with the wheels off the ground and the motors off, and look for these in the window:

- The hardware interface finds Board A. If it says it could not reach Board A, the cable or the device name is wrong.
- The IMU, GPS, LiDAR and camera drivers start without repeating errors.
- The control window opens and shows a camera picture.
- Choosing Automatic brings up the arming check. Without a surveyed course file it refuses, which is correct.

## If the icon does nothing

Right-click it and choose Allow Launching. If that is not offered, run the script from a terminal: `bash run_robot.sh`.
