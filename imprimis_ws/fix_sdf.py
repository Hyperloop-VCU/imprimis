# When gazebo saves an sdf file, it expands all model:// paths to absolute paths, which causes parsing errors.
# It also adds imprimis to the file. Imprimis should NOT be in the world file (it's spawned in by ROS)
# This script reverses this expansion and removes imprimis from the file so it can be parsed correctly.

from pathlib import Path

# Get absolute path to sdf
script_dir = Path(__file__).parent.resolve()
file_path = script_dir / 'src/imprimis_hardware_platform/worlds/igvc.sdf'

# Line number of imprimis in the file
remove_idx = None

# Read file contents
with open(file_path, 'r') as infile:
    lines = infile.readlines()

for i, line in enumerate(lines):

    # Turn <uri>file:///home/.../imprimis/imprimis_ws/install/imprimis_hardware_platform/share/imprimis_hardware_platform/worlds/model://igvc/blue_barrel.glb</uri>
    # Into <uri>model://igvc/blue_barrel.glb</uri>
    if line.strip().startswith("<uri>file:///"):
        lines[i] = "<uri>" + line.split("worlds/")[1]

    
    # Remove this part:
    # <include>
    #  <uri>file://<urdf-string></uri>
    #  <name>imprimis</name>
    #  <pose>0.00086801768763651534 -0.00040509181222155483 -4.8624507039019225 0.003577517602332199 0.0074979657887728843 -1.5448191486459076e-05</pose>
    # </include>
    # This code assumes only one urdf-string exists, and it's imprimis
    elif line.strip() == "<uri>file://<urdf-string></uri>":
        remove_idx = i - 1

if remove_idx is not None:
    for _ in range(5):
        lines.pop(remove_idx)

# Overwrite file with new contents
with open(file_path, 'w') as outfile:
    outfile.writelines(lines)
