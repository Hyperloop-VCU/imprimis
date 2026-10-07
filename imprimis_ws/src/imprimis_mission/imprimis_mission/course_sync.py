"""Keep the simulator's IGVC course in step with the Blender map.

The Blender file is the master copy of the course. This module reads it (by running Blender without
a window) and writes the three things the simulator uses:

    worlds/igvc2027.sdf                    where every barrel and the ramp stand
    imprimis_mission/courses/igvc2027.json the same course in numbers, for the lap manager and the control window
    meshes/igvc2027_course, igvc2027_ramp  the ground with its lane lines, and the ramp (only when they changed)

Two ways to use it.

  Before a launch (run_sim.sh does this):
      python3 course_sync.py --blend "<map>.blend" --repo "<Gethub/imprimis>"
    It does nothing when the map has not been saved since the last time.

  While the simulator runs: the node course_watcher (main() below) looks at the map file every two
  seconds. When you save the map in Blender, it moves, adds and removes barrels in the running
  simulator, moves the ramp, clears the cost maps, and tells the lap manager and the control window.
  Changed lane lines are written for the next start; the running simulator cannot reload its ground.
"""
import argparse
import glob
import hashlib
import json
import math
import os
import shutil
import subprocess
import sys
import time

WORLD_REL = 'imprimis_ws/src/imprimis_hardware_platform/worlds/igvc2027.sdf'
MESH_REL = 'imprimis_ws/src/imprimis_hardware_platform/meshes'
COURSE_REL = 'imprimis_ws/src/imprimis_mission/courses/igvc2027.json'
STAMP_REL = 'imprimis_ws/src/imprimis_mission/courses/igvc2027.sync.json'
SCRIPT_REL = 'imprimis_ws/src/imprimis_mission/blender/export_course.py'
COLORS = ('orange', 'yellow', 'red', 'green', 'black')
FIVE_FEET = 1.524


# ---------------------------------------------------------------------------------- Blender
def find_blender():
    env = os.environ.get('IMPRIMIS_BLENDER')
    if env and os.path.isfile(env):
        return env
    found = shutil.which('blender')
    if found:
        return found
    roots = ['/mnt/c/Program Files/Blender Foundation', r'C:\Program Files\Blender Foundation']
    hits = []
    for root in roots:
        hits += glob.glob(os.path.join(root, 'Blender *', 'blender.exe'))
    return sorted(hits)[-1] if hits else None


def native(path, exe):
    """A path the Blender program understands. Windows Blender started from Ubuntu needs Windows paths."""
    if os.name != 'nt' and exe.lower().endswith('.exe'):
        return subprocess.run(['wslpath', '-w', path], capture_output=True, text=True, check=True).stdout.strip()
    return path


def export_from_blender(blend, repo, work, with_meshes=False):
    """Run Blender on the map. Returns the course as a dict. work is a scratch folder on the same drive as repo."""
    exe = find_blender()
    if exe is None:
        raise RuntimeError('Blender was not found. Install Blender, or set IMPRIMIS_BLENDER to the full path of blender.exe.')
    os.makedirs(work, exist_ok=True)
    out = os.path.join(work, 'course_from_blender.json')
    if os.path.exists(out):
        os.remove(out)
    cmd = [exe, '--background', native(blend, exe), '--python', native(os.path.join(repo, SCRIPT_REL), exe), '--', native(out, exe)]
    if with_meshes:
        cmd.append(native(os.path.join(work, 'meshes'), exe))
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=180)
    if not os.path.isfile(out):
        raise RuntimeError('Blender did not write the course. Its last lines:\n' + '\n'.join((result.stdout + result.stderr).splitlines()[-8:]))
    with open(out, encoding='utf-8') as f:
        return json.load(f)


# ---------------------------------------------------------------------------------- outputs
def layout_id(course):
    h = hashlib.sha1()
    for b in sorted(course['barrels'], key=lambda b: (b['xy'][0], b['xy'][1])):
        h.update(('%s %.2f %.2f;' % (b['color'], b['xy'][0], b['xy'][1])).encode())
    h.update(json.dumps(course.get('ramp'), sort_keys=True).encode())
    h.update(str(course.get('surface_fingerprint')).encode())
    return h.hexdigest()[:8]


def thin(line, step):
    out = [line[0]]
    for p in line[1:]:
        if math.dist(p, out[-1]) >= step:
            out.append(p)
    if out[-1] != line[-1]:
        out.append(line[-1])
    return [[round(p[0], 2), round(p[1], 2)] for p in out]


def narrow_passages(course, limit=FIVE_FEET, wall=0.45):
    """Pairs of barrels whose gap is too small to drive through by the rules (5 ft) but too wide to be a wall."""
    r = course['barrel']['diameter'] / 2.0
    b = course['barrels']
    found = []
    for i in range(len(b)):
        for j in range(i + 1, len(b)):
            gap = math.dist(b[i]['xy'], b[j]['xy']) - 2 * r
            if wall < gap < limit:
                found.append((round(gap, 2), b[i]['name'], b[j]['name']))
    return sorted(found)


def mission_course(course, previous=None):
    """The course file the lap manager and the control window read."""
    wps = course.get('waypoints', [])
    centerline = (previous or {}).get('centerline', [])

    def arm(name, south):
        # a checkpoint halfway along an arm, so that a lap cannot count if the robot cut across the infield
        side = [q for q in centerline if (q[0] > 8.0 if south else q[0] < -20.0)]
        if not side:
            return []
        mid = (min(q[1] for q in side) + max(q[1] for q in side)) / 2.0
        q = min(side, key=lambda q: abs(q[1] - mid))
        # the radius allows for the 2 m that odometry has drifted by the time the robot reaches the north arm
        return [{'name': name, 'x': round(q[0], 3), 'y': round(q[1], 3), 'radius': 4.5}]

    checkpoints = [{'name': 'South straight (speed check)', 'x': 13.411, 'y': 0.0, 'radius': 3.0}]
    checkpoints += arm('South arm', True)
    for i, w in enumerate(wps, 1):
        checkpoints.append({'name': 'Waypoint %d' % i, 'x': w[0], 'y': w[1], 'radius': 3.0})
    checkpoints += arm('North arm', False)
    checkpoints.append({'name': 'North start (before the ramp)', 'x': -12.505, 'y': 0.0, 'radius': 3.0})
    return {
        'name': 'igvc2027',
        'description': 'IGVC 2027 AutoNav course. Written by course_sync.py from the Blender map "%s". Do not edit by hand; '
                       'edit the map in Blender and save it. Origin at the South Start line, +X is the direction of travel.' % course.get('source', ''),
        'layout_id': layout_id(course),
        'surface_fingerprint': course.get('surface_fingerprint'),
        'length': (previous or {}).get('length', 152.4),
        'finish_line': [[0.0, -2.6], [0.0, 2.6]],
        'travel_direction': [1.0, 0.0],
        'checkpoints': checkpoints,
        'ramp': course.get('ramp'),
        'barrel_radius': course['barrel']['diameter'] / 2.0,
        'barrel_height': course['barrel']['height'],
        'barrels': [{'name': b['name'], 'xy': [round(b['xy'][0], 3), round(b['xy'][1], 3)], 'color': b['color']} for b in course['barrels']],
        'lane_lines': [thin(line, 0.6) for line in course.get('lane_lines', []) if len(line) > 1],
        'potholes': course.get('potholes', []),
        # Where the course is on the earth. In the simulator the datum is the origin of the Gazebo world (see
        # spherical_coordinates in world_sdf below) and the world's X axis points east. For a real course these
        # numbers are surveyed: the middle of the start line, and the compass bearing of the first straight.
        'gps': (previous or {}).get('gps') or {
            'datum_lat': 42.66791, 'datum_lon': -83.21958, 'x_axis_bearing_deg': 90.0,
            'start_box': [-4.0, 14.0, 3.0],          # x from, x to, and half width in y, meters: the start straight
            'boundary_half_width_m': 6.0},           # how far from the course line the robot may be
        'centerline': (previous or {}).get('centerline', []),
    }


def barrel_include(b):
    return ('    <include>\n      <uri>model://igvc2027_barrel_%s</uri>\n      <static>true</static>\n      <name>%s</name>\n'
            '      <pose>%.3f %.3f 0 0 0 0</pose>\n    </include>' % (b['color'], b['name'], b['xy'][0], b['xy'][1]))


def world_sdf(course):
    r = course.get('ramp')
    ramp = ''
    if r:
        ramp = ('    <include>\n      <uri>model://igvc2027_ramp</uri>\n      <name>ramp</name>\n'
                '      <pose>%.3f %.3f 0 0 0 %.4f</pose>\n    </include>\n\n' % (r['center'][0], r['center'][1], r.get('yaw', 0.0)))
    ex = course.get('extent', {'x': [-30, 18], 'y': [-40, 2]})
    return '''<?xml version="1.0"?>
<sdf version="1.9">
  <world name="default">
    <!-- IGVC 2027 AutoNav course. Written by course_sync.py from the Blender map "%s".
         Do not edit by hand: move things in Blender and save; the simulator follows.
         Frame: origin at the South Start line, +X is the direction of travel (course south), +Y is course east.
         The robot spawns at the origin facing +X. All barrels are static (see the note in igvc2.sdf). -->
    <scene>
      <ambient>0.8 0.8 0.8 1</ambient>
      <background>0.6 0.75 0.9 1</background>
      <grid>false</grid>
      <shadows>1</shadows>
    </scene>

    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>42.66791</latitude_deg>
      <longitude_deg>-83.21958</longitude_deg>
      <elevation>286</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>

    <physics name="1ms" type="ODE">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>

    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <include>
      <uri>model://igvc2027_course</uri>
      <name>course</name>
      <pose>0 0 0 0 0 0</pose>
    </include>

%s%s

    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>%.1f %.1f 45 0 0.75 2.6</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>
  </world>
</sdf>
''' % (course.get('source', ''), ramp, '\n'.join(barrel_include(b) for b in course['barrels']), ex['x'][1] + 25, ex['y'][1] + 18)


def fix_materials(folder):
    """Blender writes 'Ka 1 1 1'; the simulated sensors then see every untextured surface as white. Use Ka = Kd.
    Also make sure the asphalt texture is named without a path."""
    for root, _, files in os.walk(folder):
        for fn in files:
            if not fn.endswith('.mtl'):
                continue
            path = os.path.join(root, fn)
            with open(path, encoding='utf-8') as f:
                blocks = f.read().split('newmtl ')
            out = [blocks[0]]
            for b in blocks[1:]:
                lines = b.splitlines()
                kd = [l for l in lines if l.startswith('Kd ')]
                fixed = []
                for l in lines:
                    if l.startswith('Ka ') and kd:
                        l = 'Ka ' + kd[0][3:]
                    if l.startswith('map_Kd '):
                        l = 'map_Kd ' + os.path.basename(l[7:].strip().replace('\\', '/'))
                    fixed.append(l)
                out.append('\n'.join(fixed) + '\n')
            with open(path, 'w', encoding='utf-8', newline='\n') as f:
                f.write('newmtl '.join(out))


def write_text(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', encoding='utf-8', newline='\n') as f:
        f.write(text)


def load_json(path):
    if os.path.isfile(path):
        with open(path, encoding='utf-8') as f:
            return json.load(f)
    return None


def old_barrels(previous):
    """Barrels of the course file on disk, with the names the simulator knows them by."""
    out = []
    for i, b in enumerate((previous or {}).get('barrels', []), 1):
        out.append({'name': b.get('name') or 'barrel_%03d_%s' % (i, b.get('color', 'orange')), 'color': b.get('color', 'orange'), 'xy': b['xy']})
    return out


def compare(previous, course):
    """What changed between the course on disk and the Blender map."""
    old = {b['name']: b for b in old_barrels(previous)}
    new = {b['name']: b for b in course['barrels']}
    moved = [new[n] for n in new if n in old and (math.dist(new[n]['xy'], old[n]['xy']) > 0.005 or new[n]['color'] != old[n]['color'])]
    added = [new[n] for n in new if n not in old]
    removed = [old[n] for n in old if n not in new]
    ramp_old, ramp_new = (previous or {}).get('ramp'), course.get('ramp')
    def key(r):
        return None if not r else (round(r['center'][0], 3), round(r['center'][1], 3), round(r.get('yaw', 0.0), 3))
    def size(r):
        return None if not r else (round(r['length'], 2), round(r['width'], 2), round(r['rise'], 2))
    return {'moved': moved, 'added': added, 'removed': removed,
            'recolored': [n for n in new if n in old and new[n]['color'] != old[n]['color']],
            'ramp_moved': key(ramp_old) != key(ramp_new), 'ramp_resized': size(ramp_old) != size(ramp_new),
            'surface_changed': (previous or {}).get('surface_fingerprint') != course.get('surface_fingerprint'),
            'waypoints_changed': [c for c in (previous or {}).get('checkpoints', []) if c['name'].startswith('Waypoint')]
                                 != [c for c in mission_course(course, previous)['checkpoints'] if c['name'].startswith('Waypoint')]}


def describe(change):
    parts = []
    for word, key in (('moved', 'moved'), ('added', 'added'), ('removed', 'removed')):
        if change[key]:
            n = len(change[key])
            parts.append('%d barrel%s %s' % (n, '' if n == 1 else 's', word))
    if change['ramp_moved']:
        parts.append('ramp moved')
    if change['ramp_resized']:
        parts.append('ramp resized')
    if change['surface_changed']:
        parts.append('lane lines or potholes changed')
    if change['waypoints_changed']:
        parts.append('waypoints moved')
    return ', '.join(parts) if parts else 'no change'


def sync(blend, repo, extra_copies=(), force=False, log=print):
    """Bring the simulator files up to date with the Blender map.

    Returns (course file contents, change dict) or (None, None) when the map has not been saved since
    the last sync. extra_copies are other source trees that get the same files (the Ubuntu workspace).
    """
    stamp_path = os.path.join(repo, STAMP_REL)
    stamp = load_json(stamp_path) or {}
    st = os.stat(blend)
    if not force and stamp.get('blend_mtime') == st.st_mtime and stamp.get('blend_size') == st.st_size \
            and os.path.isfile(os.path.join(repo, WORLD_REL)):
        return None, None
    work = os.path.join(repo, '..', 'simulator-setup', '.course-sync')
    course = export_from_blender(blend, repo, work)
    previous = load_json(os.path.join(repo, COURSE_REL))
    change = compare(previous, course)
    if change['surface_changed'] or change['ramp_resized']:
        course = export_from_blender(blend, repo, work, with_meshes=True)
        fix_materials(os.path.join(work, 'meshes'))
    mission = mission_course(course, previous)
    for root in (repo,) + tuple(extra_copies):
        write_text(os.path.join(root, WORLD_REL), world_sdf(course))
        write_text(os.path.join(root, COURSE_REL), json.dumps(mission, indent=1))
        if change['surface_changed'] or change['ramp_resized']:
            for model, wanted in (('igvc2027_course', change['surface_changed']), ('igvc2027_ramp', change['ramp_resized'])):
                src = os.path.join(work, 'meshes', model, 'meshes')
                if wanted and os.path.isdir(src):
                    dst = os.path.join(root, MESH_REL, model, 'meshes')
                    os.makedirs(dst, exist_ok=True)
                    for fn in os.listdir(src):
                        shutil.copy(os.path.join(src, fn), os.path.join(dst, fn))
    write_text(stamp_path, json.dumps({'blend': os.path.basename(blend), 'blend_mtime': st.st_mtime, 'blend_size': st.st_size,
                                       'layout_id': mission['layout_id'], 'barrels': len(course['barrels']),
                                       'synced': time.strftime('%Y-%m-%d %H:%M:%S'), 'change': describe(change)}, indent=1))
    log('Course read from the Blender map: %d barrels, layout %s (%s).' % (len(course['barrels']), mission['layout_id'], describe(change)))
    for note in course.get('notes', []):
        log('Note: ' + note)
    tight = narrow_passages(course)
    if tight:
        log('Note: %d gaps between barrels are narrower than 5 ft (1.52 m), the narrowest %.2f m between %s and %s.'
            % (len(tight), tight[0][0], tight[0][1], tight[0][2]))
    return mission, change


# ---------------------------------------------------------------------------------- live changes in Gazebo
def gz(service, reqtype, req, world='default', timeout=3000):
    cmd = ['gz', 'service', '-s', '/world/%s/%s' % (world, service), '--reqtype', reqtype, '--reptype', 'gz.msgs.Boolean',
           '--timeout', str(timeout), '--req', req]
    r = subprocess.run(cmd, capture_output=True, text=True)
    return 'true' in r.stdout


def apply_live(change, course, world='default', log=print):
    """Carry a change into the running simulator. Returns the number of things done."""
    done = 0
    recolored = set(change.get('recolored', []))
    for b in change['removed'] + [b for b in change['moved'] if b['name'] in recolored]:
        done += gz('remove', 'gz.msgs.Entity', 'name: "%s", type: MODEL' % b['name'], world)
    for b in change['moved']:
        if b['name'] not in recolored:
            done += gz('set_pose', 'gz.msgs.Pose', 'name: "%s", position: {x: %.3f, y: %.3f, z: 0}' % (b['name'], b['xy'][0], b['xy'][1]), world)
    for b in change['added'] + [b for b in change['moved'] if b['name'] in recolored]:
        sdf = ("<sdf version='1.9'><model name='%s'><static>true</static><include><uri>model://igvc2027_barrel_%s</uri>"
               "<name>body</name></include></model></sdf>" % (b['name'], b['color']))
        done += gz('create', 'gz.msgs.EntityFactory', 'sdf: "%s", name: "%s", pose: {position: {x: %.3f, y: %.3f, z: 0}}'
                   % (sdf, b['name'], b['xy'][0], b['xy'][1]), world)
    if change['ramp_moved'] and course.get('ramp'):
        r = course['ramp']
        yaw = r.get('yaw', 0.0)
        done += gz('set_pose', 'gz.msgs.Pose', 'name: "ramp", position: {x: %.3f, y: %.3f, z: 0}, orientation: {z: %.5f, w: %.5f}'
                   % (r['center'][0], r['center'][1], math.sin(yaw / 2), math.cos(yaw / 2)), world)
    return done


# ---------------------------------------------------------------------------------- the watcher node
def main(args=None):
    import rclpy
    from nav2_msgs.srv import ClearEntireCostmap
    from rclpy.node import Node
    from std_msgs.msg import String

    class CourseWatcher(Node):
        def __init__(self):
            super().__init__('course_watcher')
            self.blend = self.declare_parameter('blend_file', '').value
            self.repo = self.declare_parameter('repo_src', '').value
            self.workspace = self.declare_parameter('workspace_src', '').value
            self.world = self.declare_parameter('world_name', 'default').value
            self.period = self.declare_parameter('poll_period', 2.0).value
            self.pub = self.create_publisher(String, 'course/changed', 5)
            self.clear = [self.create_client(ClearEntireCostmap, 'global_costmap/clear_entirely_global_costmap'),
                          self.create_client(ClearEntireCostmap, 'local_costmap/clear_entirely_local_costmap')]
            self.seen = None
            if not (self.blend and os.path.isfile(self.blend) and os.path.isdir(self.repo)):
                self.get_logger().info('No Blender map to watch (blend_file "%s").' % self.blend)
                return
            self.seen = os.stat(self.blend).st_mtime
            self.create_timer(self.period, self.check)
            self.get_logger().info('Watching the Blender map %s. Save it in Blender and the simulator follows.' % os.path.basename(self.blend))

        def check(self):
            try:
                st = os.stat(self.blend)
            except OSError:
                return
            if st.st_mtime == self.seen or time.time() - st.st_mtime < 1.0:   # unchanged, or still being written
                return
            self.seen = st.st_mtime
            try:
                extra = (self.workspace,) if self.workspace and os.path.isdir(self.workspace) else ()
                mission, change = sync(self.blend, self.repo, extra, force=True, log=lambda s: self.get_logger().info(s))
            except Exception as e:
                self.get_logger().error('Could not read the Blender map: %s' % e)
                self.pub.publish(String(data=json.dumps({'ok': False, 'text': 'COURSE NOT UPDATED: ' + str(e)[:80]})))
                return
            text = describe(change)
            if text == 'no change':
                return
            done = apply_live(change, mission, self.world)
            for client in self.clear:       # the cost maps still hold the barrels where they used to stand
                if client.service_is_ready():
                    client.call_async(ClearEntireCostmap.Request())
            note = ''
            if change['surface_changed'] or change['ramp_resized']:
                note = ' Lane lines and the ramp shape are redrawn at the next start of the simulator.'
            self.get_logger().info('Course updated from Blender: %s (%d changes made in the running simulator).%s' % (text, done, note))
            self.pub.publish(String(data=json.dumps({'ok': True, 'text': 'COURSE UPDATED: ' + text, 'layout_id': mission['layout_id']})))

    rclpy.init(args=args)
    node = CourseWatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description='Bring the simulator course up to date with the Blender map.')
    ap.add_argument('--blend', required=True)
    ap.add_argument('--repo', required=True, help='the Gethub/imprimis folder')
    ap.add_argument('--copy', action='append', default=[], help='another source tree that gets the same files')
    ap.add_argument('--force', action='store_true')
    a = ap.parse_args()
    if not os.path.isfile(a.blend):
        print('Blender map not found: %s (the course is left as it is).' % a.blend)
        sys.exit(0)
    try:
        mission, change = sync(a.blend, a.repo, tuple(a.copy), a.force)
        if mission is None:
            print('The course is up to date with the Blender map.')
    except Exception as e:      # a launch must not fail because Blender is missing
        print('Could not read the Blender map, so the course is left as it is: %s' % e)
