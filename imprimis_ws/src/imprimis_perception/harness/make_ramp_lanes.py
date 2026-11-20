"""Write sim/ramp_lanes/meshes/ramp_lanes.obj: the two lane lines carried over the igvc2027 ramp.

The mesh is in the ramp's frame; the launch file spawns it at the ramp's pose.
Run: python3 harness/make_ramp_lanes.py
"""
from pathlib import Path

RAMP_X = -6.386
RAMP_HALF_LENGTH = 2.976
RAMP_PEAK = 0.446
TAPE_Z = 0.004
LINES_Y = [(1.922, 2.024), (-2.057, -1.955)]
LINE_X = [-9.763, -9.362, -6.386, -3.410, -3.009]

OUT = Path(__file__).resolve().parent.parent / 'sim' / 'ramp_lanes' / 'meshes' / 'ramp_lanes.obj'


def ramp_height(x):
    return RAMP_PEAK * max(0.0, 1.0 - abs(x) / RAMP_HALF_LENGTH)


def main():
    lines = ['mtllib ramp_lanes.mtl', 'o ramp_lanes', 'usemtl white_tape']
    faces = []
    count = 0
    for y0, y1 in LINES_Y:
        first = count + 1
        for xw in LINE_X:
            x = xw - RAMP_X
            z = ramp_height(x) + TAPE_Z
            lines.append(f'v {x:.4f} {y0:.4f} {z:.4f}')
            lines.append(f'v {x:.4f} {y1:.4f} {z:.4f}')
            count += 2
        for i in range(len(LINE_X) - 1):
            a = first + 2 * i
            faces.append(f'f {a} {a + 2} {a + 3} {a + 1}')
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text('\n'.join(lines + faces) + '\n')
    print(f'wrote {OUT}')


if __name__ == '__main__':
    main()
