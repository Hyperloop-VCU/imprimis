"""Lap logic and lap memory. No ROS imports, so it can be tested offline.

Course      what a course file describes: the finish line, the checkpoints, and drawing data.
LapTracker  follows the robot's position and decides when a lap has started, which checkpoints have
            been passed, and when the finish line has been crossed.
RouteMemory keeps every run on disk and remembers the best completed lap. The next automatic run
            drives the route of that best lap. This is the whole of the "learning": the robot repeats
            and refines what has worked. There is no neural network in it.
"""
import csv
import json
import math
import os
import time

# Outline of the robot seen from above, from base_link (the drive axle). Same as the Nav2 footprint.
FOOTPRINT = [(0.22, 0.49), (0.22, -0.49), (-0.20, -0.49), (-0.86, -0.33), (-0.86, 0.33), (-0.20, 0.49)]


class Course:
    def __init__(self, data):
        self.name = data.get('name', 'course')
        self.layout_id = data.get('layout_id', 'original')   # changes when a barrel, the ramp or a lane line is moved
        self.surface_fingerprint = data.get('surface_fingerprint')   # changes when a lane line or a pothole is moved
        self.finish_line = [tuple(p) for p in data['finish_line']]        # two end points
        self.travel_direction = tuple(data.get('travel_direction', (1.0, 0.0)))
        self.checkpoints = data.get('checkpoints', [])                    # dicts: name, x, y, radius
        self.length = float(data.get('length', 0.0))                      # loop length in meters
        self.lane_lines = data.get('lane_lines', [])
        self.barrels = data.get('barrels', [])
        self.barrel_radius = float(data.get('barrel_radius', 0.3))
        self.potholes = data.get('potholes', [])                          # dicts: xy, diameter
        # Where the course is on the earth: datum_lat, datum_lon (the middle of the start line), x_axis_bearing_deg
        # (compass bearing of the direction of travel at the start), start_box, boundary_half_width_m. None if not surveyed.
        self.gps = data.get('gps')
        self.ramp = data.get('ramp')
        self.centerline = data.get('centerline', [])

    @staticmethod
    def load(path):
        with open(path, encoding='utf-8') as f:
            return Course(json.load(f))


def gps_to_course(latitude, longitude, gps):
    """Course coordinates (x along the direction of travel at the start, y to its left) of a GPS position.

    The course is a few hundred meters across, so the earth is taken as flat around the datum, with the two
    radii of curvature of the WGS 84 ellipsoid at that latitude. The error over 100 m is under a millimeter.
    """
    lat0 = math.radians(gps['datum_lat'])
    a, e2 = 6378137.0, 0.00669437999014
    w = math.sqrt(1.0 - e2 * math.sin(lat0) ** 2)
    north = math.radians(latitude - gps['datum_lat']) * a * (1.0 - e2) / w ** 3
    east = math.radians(longitude - gps['datum_lon']) * a / w * math.cos(lat0)
    bearing = math.radians(gps.get('x_axis_bearing_deg', 90.0))
    return east * math.sin(bearing) + north * math.cos(bearing), -east * math.cos(bearing) + north * math.sin(bearing)


def distance_to_loop(p, line):
    """Distance from point p to a closed line given as a list of points."""
    if not line:
        return None
    n = len(line)
    return min(_point_segment_distance(p, line[i], line[(i + 1) % n]) for i in range(n))


def _segments_cross(p, q, a, b):
    """True if segment p-q crosses segment a-b."""
    def side(u, v, w):
        return (v[0] - u[0]) * (w[1] - u[1]) - (v[1] - u[1]) * (w[0] - u[0])
    d1, d2 = side(a, b, p), side(a, b, q)
    d3, d4 = side(p, q, a), side(p, q, b)
    return (d1 * d2 < 0) and (d3 * d4 < 0)


def _point_segment_distance(p, a, b):
    ax, ay = b[0] - a[0], b[1] - a[1]
    t = ((p[0] - a[0]) * ax + (p[1] - a[1]) * ay) / max(1e-12, ax * ax + ay * ay)
    t = max(0.0, min(1.0, t))
    return math.hypot(p[0] - (a[0] + t * ax), p[1] - (a[1] + t * ay))


def _inside(p, polygon):
    """True if point p is inside the polygon (ray casting)."""
    inside = False
    n = len(polygon)
    for i in range(n):
        (x0, y0), (x1, y1) = polygon[i], polygon[(i + 1) % n]
        if (y0 > p[1]) != (y1 > p[1]) and p[0] < x0 + (p[1] - y0) * (x1 - x0) / (y1 - y0):
            inside = not inside
    return inside


def outline_clearance(x, y, yaw, center, radius):
    """Gap between the robot's outline and a round obstacle. Negative means overlap."""
    c, s = math.cos(yaw), math.sin(yaw)
    pts = [(x + c * fx - s * fy, y + s * fx + c * fy) for fx, fy in FOOTPRINT]
    d = min(_point_segment_distance(center, pts[i], pts[(i + 1) % len(pts)]) for i in range(len(pts)))
    return d - radius


class LapTracker:
    """States: ready -> running -> finished, or -> failed."""

    def __init__(self, course, trace_step=0.25):
        self.course = course
        self.trace_step = trace_step
        self.reset()

    def reset(self):
        self.state = 'ready'
        self.start_time = None
        self.end_time = None
        self.start_xy = None
        self.last_xy = None
        self.distance = 0.0
        self.next_checkpoint = 0
        self.checkpoint_miss = []      # closest approach to each checkpoint
        self.trace = []                # [t, x, y, yaw, speed]
        self.min_clearance = None
        self.max_speed = 0.0
        self.events = []
        self.fail_reason = None
        # scoring: done with the simulator's true pose when there is one, otherwise with odometry
        self.scored_with_truth = False
        self.score_xy = None
        self.barrel_contacts = 0
        self.lane_touches = 0
        self.touching_barrel = False
        self.touching_lane = False
        self.pothole_touches = 0
        self.touching_pothole = False
        self.true_trace = []           # [t, x, y, yaw] from the simulator
        self.lane_seen = {}            # 0.2 m cell -> number of camera frames that put a lane point in it
        self.odometry_error = None     # distance between odometry and the true pose, latest

    @property
    def lap_time(self):
        if self.start_time is None:
            return 0.0
        return (self.end_time if self.end_time is not None else self._now) - self.start_time

    def note(self, t, kind, **details):
        e = {'t': round(t - (self.start_time or t), 2), 'event': kind}
        e.update(details)
        self.events.append(e)

    def score(self, t, x, y, yaw, truth):
        """Count barrel contacts and lane line touches at one pose of the robot's outline."""
        if self.score_xy is not None and math.hypot(x - self.score_xy[0], y - self.score_xy[1]) < 0.05:
            return
        self.score_xy = (x, y)
        self.scored_with_truth = truth
        c, s = math.cos(yaw), math.sin(yaw)
        outline = [(x + c * fx - s * fy, y + s * fx + c * fy) for fx, fy in FOOTPRINT]
        nearest = None
        for barrel in self.course.barrels:
            bx, by = barrel['xy']
            if abs(bx - x) < 2.0 and abs(by - y) < 2.0:
                d = min(_point_segment_distance((bx, by), outline[i], outline[(i + 1) % len(outline)])
                        for i in range(len(outline))) - self.course.barrel_radius
                if nearest is None or d < nearest:
                    nearest = d
        if nearest is not None and (self.min_clearance is None or nearest < self.min_clearance):
            self.min_clearance = nearest
        touching = nearest is not None and nearest < 0.0
        if touching and not self.touching_barrel:
            self.barrel_contacts += 1
            self.note(t, 'barrel_contact', x=round(x, 2), y=round(y, 2))
        self.touching_barrel = touching
        on_line = False
        for line in self.course.lane_lines:
            for i in range(len(line) - 1):
                a, b = line[i], line[i + 1]
                if min(abs(a[0] - x), abs(b[0] - x)) > 1.6 or min(abs(a[1] - y), abs(b[1] - y)) > 1.6:
                    continue
                if any(_segments_cross(a, b, outline[k], outline[(k + 1) % len(outline)]) for k in range(len(outline))):
                    on_line = True
                    break
            if on_line:
                break
        if on_line and not self.touching_lane:
            self.lane_touches += 1
            self.note(t, 'lane_line_touch', x=round(x, 2), y=round(y, 2))
        self.touching_lane = on_line
        # a pothole counts when any part of the robot's outline is over it
        on_hole = False
        for hole in self.course.potholes:
            hx, hy = hole['xy']
            if abs(hx - x) > 2.0 or abs(hy - y) > 2.0:
                continue
            gap = min(_point_segment_distance((hx, hy), outline[i], outline[(i + 1) % len(outline)]) for i in range(len(outline)))
            if gap < hole.get('diameter', 0.61) / 2.0 or _inside((hx, hy), outline):
                on_hole = True
                break
        if on_hole and not self.touching_pothole:
            self.pothole_touches += 1
            self.note(t, 'pothole_touch', x=round(x, 2), y=round(y, 2))
        self.touching_pothole = on_hole

    def update(self, t, x, y, yaw, speed, truth=None):
        """Feed one pose. Returns a list of things that just happened: 'started', 'checkpoint', 'finished'.

        x, y, yaw are the robot's own estimate (odometry); the lap logic runs on them, as it must on
        the real robot. truth is the simulator's true (x, y, yaw) when available; scoring uses it.
        """
        self._now = t
        happened = []
        if self.state in ('finished', 'failed'):
            return happened
        if self.start_xy is None:
            self.start_xy = (x, y)
            self.last_xy = (x, y)
            self.trace_xy = (x, y)
            return happened
        step = math.hypot(x - self.last_xy[0], y - self.last_xy[1])
        if self.state == 'ready':
            if math.hypot(x - self.start_xy[0], y - self.start_xy[1]) > 0.3:
                self.state = 'running'
                self.start_time = t
                self.trace.append([0.0, round(self.start_xy[0], 3), round(self.start_xy[1], 3), round(yaw, 3), 0.0])
                happened.append('started')
            else:
                self.last_xy = (x, y)
                return happened
        if step > 5.0:           # a jump: the simulator was reset or the robot was moved by hand
            self.last_xy = (x, y)
            return happened
        self.distance += step
        self.max_speed = max(self.max_speed, abs(speed))
        cps = self.course.checkpoints
        if self.next_checkpoint < len(cps):
            cp = cps[self.next_checkpoint]
            d = math.hypot(x - cp['x'], y - cp['y'])
            if len(self.checkpoint_miss) <= self.next_checkpoint:
                self.checkpoint_miss.append(d)
            self.checkpoint_miss[self.next_checkpoint] = min(self.checkpoint_miss[self.next_checkpoint], d)
            if d <= cp.get('radius', 2.0):
                self.note(t, 'checkpoint', name=cp.get('name', str(self.next_checkpoint)))
                self.next_checkpoint += 1
                happened.append('checkpoint')
        # finish line, crossed in the direction of travel, after every checkpoint
        a, b = self.course.finish_line
        moved = (x - self.last_xy[0], y - self.last_xy[1])
        forward = moved[0] * self.course.travel_direction[0] + moved[1] * self.course.travel_direction[1]
        enough = self.distance > 0.5 * self.course.length if self.course.length else self.distance > 5.0
        if (self.next_checkpoint >= len(cps) and enough and forward > 0
                and _segments_cross(self.last_xy, (x, y), a, b)):
            self.state = 'finished'
            self.end_time = t
            happened.append('finished')
        if truth is not None:
            self.score(t, truth[0], truth[1], truth[2], True)
            self.odometry_error = math.hypot(truth[0] - x, truth[1] - y)
        else:
            self.score(t, x, y, yaw, False)
        if math.hypot(x - self.trace_xy[0], y - self.trace_xy[1]) >= self.trace_step or happened:
            self.trace.append([round(t - self.start_time, 2), round(x, 3), round(y, 3), round(yaw, 3), round(speed, 3)])
            self.trace_xy = (x, y)
            if truth is not None:
                self.true_trace.append([round(t - self.start_time, 2), round(truth[0], 3), round(truth[1], 3), round(truth[2], 3)])
        self.last_xy = (x, y)
        return happened

    LANE_CELL = 0.2

    def add_lane_points(self, points):
        """Remember where the lane mapper saw tape (points in the odometry frame) during this lap."""
        if self.state != 'running':
            return
        for key in {(int(round(x / self.LANE_CELL)), int(round(y / self.LANE_CELL))) for x, y in points}:
            self.lane_seen[key] = self.lane_seen.get(key, 0) + 1

    def lane_memory(self):
        """Cells seen in at least two frames, as [x, y] in meters."""
        return [[round(kx * self.LANE_CELL, 2), round(ky * self.LANE_CELL, 2)] for (kx, ky), n in self.lane_seen.items() if n >= 2]

    def fail(self, t, reason):
        if self.state in ('finished', 'failed'):
            return
        self.state = 'failed'
        self.end_time = t
        self.fail_reason = reason

    def record(self, mode, extra=None):
        rec = {
            'course': self.course.name,
            'when': time.strftime('%Y-%m-%d %H:%M:%S'),
            'mode': mode,
            'result': 'success' if self.state == 'finished' else ('failed' if self.state == 'failed' else 'incomplete'),
            'fail_reason': self.fail_reason,
            'lap_time_s': round(self.lap_time, 2),
            'distance_m': round(self.distance, 2),
            'max_speed_mps': round(self.max_speed, 2),
            'average_speed_mph': round(2.237 * self.distance / self.lap_time, 2) if self.lap_time > 0 else 0.0,
            'max_speed_mph': fastest_second_mph({'trace': self.trace, 'true_trace': self.true_trace}),
            'peak_odometry_speed_mph': round(2.237 * self.max_speed, 2),
            'checkpoints_passed': self.next_checkpoint,
            'checkpoints_total': len(self.course.checkpoints),
            'checkpoint_closest_m': [round(v, 2) for v in self.checkpoint_miss],
            'scored_with': 'simulator true pose' if self.scored_with_truth else 'odometry (an estimate)',
            'min_barrel_clearance_m': None if self.min_clearance is None else round(self.min_clearance, 3),
            'barrel_contacts': self.barrel_contacts,
            'lane_line_touches': self.lane_touches,
            'pothole_touches': self.pothole_touches,
            'clean': self.state == 'finished' and self.barrel_contacts == 0 and self.lane_touches == 0 and self.pothole_touches == 0,
            'odometry_error_at_end_m': None if self.odometry_error is None else round(self.odometry_error, 2),
            'start_sim_time_s': None if self.start_time is None else round(self.start_time, 2),
            'events': self.events,
            'trace_columns': ['t_s', 'x_m', 'y_m', 'yaw_rad', 'speed_mps'],
            'trace': self.trace,
            'true_trace_columns': ['t_s', 'x_m', 'y_m', 'yaw_rad'],
            'true_trace': self.true_trace,
            'lane_memory': self.lane_memory(),
        }
        if extra:
            rec.update(extra)
        return rec


def resample(points, spacing):
    """Points along a polyline at even spacing. Each result is (x, y, yaw of the path there)."""
    if len(points) < 2:
        return [(p[0], p[1], 0.0) for p in points]
    out = []
    next_at = 0.0      # distance along the path of the next output point
    travelled = 0.0
    for i in range(len(points) - 1):
        x0, y0 = points[i][0], points[i][1]
        x1, y1 = points[i + 1][0], points[i + 1][1]
        seg = math.hypot(x1 - x0, y1 - y0)
        if seg < 1e-9:
            continue
        yaw = math.atan2(y1 - y0, x1 - x0)
        while next_at <= travelled + seg:
            f = (next_at - travelled) / seg
            out.append((x0 + (x1 - x0) * f, y0 + (y1 - y0) * f, yaw))
            next_at += spacing
        travelled += seg
    last = points[-1]
    if not out:
        return [(last[0], last[1], 0.0)]
    if math.hypot(out[-1][0] - last[0], out[-1][1] - last[1]) > 0.3 * spacing:
        out.append((last[0], last[1], out[-1][2]))
    return out


def fastest_second_mph(record):
    """The highest speed held for one second, in miles per hour, from the true path if the run has one and from
    odometry otherwise. The speed that odometry reports at one instant is not used: it spikes when a wheel slips."""
    trace = record.get('true_trace') or []
    if len(trace) < 10:
        trace = record.get('trace') or []
    best, j, cum = 0.0, 0, [0.0]
    for k in range(1, len(trace)):
        cum.append(cum[-1] + math.hypot(trace[k][1] - trace[k - 1][1], trace[k][2] - trace[k - 1][2]))
    for i in range(len(trace)):
        while j < i and trace[i][0] - trace[j][0] > 1.0:
            j += 1
        dt = trace[i][0] - trace[j][0]
        if dt >= 0.8:
            best = max(best, (cum[i] - cum[j]) / dt)
    return round(2.237 * best, 2)


class RouteMemory:
    """Folder layout:
         demonstrations/  laps driven by a person, used when no completed lap is on record yet
         runs/            one JSON file per run, completed or not
         best_route.json  the route the next automatic run will drive
         history.csv      one line per run
    """

    def __init__(self, folder):
        self.folder = folder
        for sub in ('', 'runs', 'demonstrations'):
            os.makedirs(os.path.join(folder, sub), exist_ok=True)
        self.best_path = os.path.join(folder, 'best_route.json')
        self.lane_path = os.path.join(folder, 'lane_memory.json')
        self.history_path = os.path.join(folder, 'history.csv')

    def load_lane_memory(self, surface_fingerprint=None):
        """Lane line cells remembered from an earlier lap, or [] if there are none for this course surface."""
        if not os.path.isfile(self.lane_path):
            return []
        with open(self.lane_path, encoding='utf-8') as f:
            data = json.load(f)
        if data.get('surface_fingerprint') not in (None, surface_fingerprint):
            return []                    # the lane lines were moved in Blender since; the memory no longer holds
        best = self.load_best()
        if best is None or best.get('run') != data.get('run'):
            return []                    # seen on another lap than the route: the two do not share a frame
        return data.get('cells', [])

    def keep_lane_memory(self, record):
        """The lane lines seen on the lap that became the best route are stored beside it."""
        cells = record.get('lane_memory') or []
        if record['result'] != 'success' or len(cells) < 100:
            return False
        with open(self.lane_path, 'w', encoding='utf-8') as f:
            json.dump({'when': record['when'], 'run': record.get('run'), 'cell_m': LapTracker.LANE_CELL,
                       'surface_fingerprint': record.get('surface_fingerprint'), 'cells': cells}, f)
        return True

    def load_best(self):
        if os.path.isfile(self.best_path):
            with open(self.best_path, encoding='utf-8') as f:
                return json.load(f)
        return None

    def newest_demonstration(self):
        folder = os.path.join(self.folder, 'demonstrations')
        files = sorted(f for f in os.listdir(folder) if f.endswith('.json'))
        if not files:
            return None
        with open(os.path.join(folder, files[-1]), encoding='utf-8') as f:
            demo = json.load(f)
        demo['file'] = files[-1]
        return demo

    def route(self):
        """The route to drive, as a dict: points, description, best_time, as_goals.

        as_goals is true when the points are few and each was a place the robot actually stood (a
        demonstration recovered from logs). They are then used as goals one by one. A recorded lap is
        dense, and goals are taken from it at even spacing.
        """
        best = self.load_best()
        sound = self.best_sound_lap()
        if sound is not None:
            if sound['mode'] == 'manual':
                own = self.own_lap_of(sound['run'])
                if own is not None:
                    return {'points': own['points'], 'best_time': (best or sound).get('lap_time_s'), 'as_goals': False,
                            'description': 'run %s, the robot\'s own clean lap (%.0f s) of the route you drove in run %s'
                                           % (own['run'], own['lap_time_s'], sound['run'])}
            if best is None or sound['run'] != best.get('run'):
                return {'points': sound['points'], 'best_time': (best or sound).get('lap_time_s'), 'as_goals': False,
                        'description': 'run %s (%s, %.0f s, clean), the best lap recorded in a sound frame' % (sound['run'], sound['mode'], sound['lap_time_s'])}
        if best:
            return {'points': best['points'], 'best_time': best.get('lap_time_s'), 'as_goals': False,
                    'description': 'best lap on record (run %s, %s, %s%s)' % (best.get('run', '?'), best.get('mode', '?'), best.get('when', '?'),
                                                                               ', clean' if best.get('clean') else '')}
        demo = self.newest_demonstration()
        if demo:
            return {'points': demo['points'], 'best_time': None, 'as_goals': bool(demo.get('points_are_goals', False)),
                    'description': 'demonstration ' + demo['file']}
        return {'points': None, 'best_time': None, 'as_goals': False, 'description': 'none'}

    @staticmethod
    def frame_is_sound(record):
        """True if the odometry frame of this lap was lined up with the course when the lap began.

        Odometry turns by about one degree per lap within a session. A lap driven seventh in a session is
        stored three degrees askew, which is 2 m at the far side of the course, and as a route in a new
        session it leads the robot into the barrels. A lap is sound if it was the first of its session, or
        if the lap manager lined its frame up with the start straight before it began.
        """
        anchor = record.get('frame_anchor')
        if anchor is not None:
            return anchor.get('source') in ('lane lines', 'fresh start') and (anchor.get('source') != 'fresh start' or record.get('lap_in_session', 1) == 1)
        start = record.get('start_sim_time_s')
        return start is not None and start < 60.0

    def clean_laps(self):
        """Clean completed laps from the history, best first: (seconds, run number, mode, route used)."""
        laps = []
        if not os.path.isfile(self.history_path):
            return laps
        with open(self.history_path, encoding='utf-8', newline='') as fh:
            for row in csv.DictReader(fh):
                if row.get('result') != 'success' or 'simulator' not in (row.get('scored_with') or ''):
                    continue
                if sum(int(float(row.get(k) or 0)) for k in ('barrel_contacts', 'lane_line_touches', 'pothole_touches')):
                    continue
                laps.append((float(row.get('lap_time_s') or 1e9), int(row['run']), row.get('mode'), row.get('route_used') or ''))
        return sorted(laps)

    def load_run(self, number, mode):
        try:
            with open(os.path.join(self.folder, 'runs', 'run_%03d_%s_success.json' % (number, mode)), encoding='utf-8') as fh:
                return json.load(fh)
        except (OSError, ValueError):
            return None

    def best_sound_lap(self):
        """The best clean lap whose frame is sound, with its path as route points, or None."""
        for seconds, number, mode, _ in self.clean_laps()[:20]:
            record = self.load_run(number, mode)
            if record is None or len(record.get('trace', [])) <= 10 or not self.frame_is_sound(record):
                continue
            dense = resample([[p[1], p[2]] for p in record['trace']], 0.5)
            return {'run': number, 'mode': mode, 'lap_time_s': seconds, 'points': [[round(p[0], 3), round(p[1], 3)] for p in dense]}
        return None

    def own_lap_of(self, hand_run):
        """The best clean automatic lap that was guided by the hand-driven lap hand_run, or None.

        A person cuts corners closer than the planner dares to, and drives a line the robot can follow only
        some of the time. Automatic laps guided straight by a fast hand-driven lap failed three times out of
        four in a test. Once the robot has got round cleanly with that lap as its guide, its own path, which
        has the planner's margins in it, is the better guide: the lesson of the hand-driven lap, digested.
        The hand-driven lap stays the best lap on record, and its time stays the time to beat.
        """
        if hand_run is None:
            return None
        markers = ('(run %s,' % hand_run, 'run you drove in run %s' % hand_run, 'route you drove in run %s' % hand_run)
        chosen, record = None, None
        for seconds, number, mode, route_used in self.clean_laps():
            if mode != 'automatic' or not any(m in route_used for m in markers):
                continue
            candidate = self.load_run(number, mode)
            if candidate is None or len(candidate.get('trace', [])) <= 10 or not self.frame_is_sound(candidate):
                continue
            chosen, record = (seconds, number), candidate
            break
        if chosen is None:
            return None
        dense = resample([[p[1], p[2]] for p in record['trace']], 0.5)
        return {'run': chosen[1], 'lap_time_s': chosen[0], 'points': [[round(p[0], 3), round(p[1], 3)] for p in dense]}

    def run_count(self):
        if not os.path.isfile(self.history_path):
            return 0
        with open(self.history_path, encoding='utf-8') as f:
            return max(0, sum(1 for _ in f) - 1)

    HISTORY_COLUMNS = ['run', 'when', 'mode', 'result', 'lap_time_s', 'distance_m', 'average_speed_mph', 'max_speed_mph', 'checkpoints',
                       'barrel_contacts', 'lane_line_touches', 'pothole_touches', 'min_barrel_clearance_m', 'scored_with',
                       'course_layout', 'route_used', 'became_best', 'fail_reason']

    @staticmethod
    def rank(record):
        """Smaller is better. A clean lap (no barrel contact, no lane line touch, no pothole under the outline) beats any lap that is
        not clean. Among laps that are not clean, fewer contacts and touches win; a lap that was not
        scored with the simulator's true position counts as the worst of these, because odometry
        cannot tell. Lap time decides the rest."""
        truth = 'simulator' in str(record.get('scored_with', ''))
        if truth:
            faults = (record.get('barrel_contacts') or 0) + (record.get('lane_line_touches') or 0) + (record.get('pothole_touches') or 0)
        else:
            faults = 999
        return (0 if (truth and faults == 0) else 1, faults, record['lap_time_s'])

    def _consider(self, record, name):
        """Make this run the best route if it is a completed lap that ranks ahead of the present best."""
        if record['result'] != 'success' or len(record['trace']) <= 10:
            return False
        best = self.load_best()
        if best is not None and best.get('rank') is not None and tuple(best['rank']) <= self.rank(record):
            return False
        dense = resample([[p[1], p[2]] for p in record['trace']], 0.5)
        with open(self.best_path, 'w', encoding='utf-8') as fh:
            json.dump({'when': record['when'], 'mode': record['mode'], 'run': record['run'],
                       'lap_time_s': record['lap_time_s'], 'clean': self.rank(record)[0] == 0,
                       'barrel_contacts': record.get('barrel_contacts'),
                       'lane_line_touches': record.get('lane_line_touches'),
                       'pothole_touches': record.get('pothole_touches'),
                       'scored_with': record.get('scored_with', 'not scored'), 'rank': list(self.rank(record)),
                       'source_file': name, 'points': [[round(p[0], 3), round(p[1], 3)] for p in dense]}, fh)
        return True

    def _history_row(self, record, new_best):
        mph = record.get('average_speed_mph')
        if mph is None:
            mph = round(2.237 * record['distance_m'] / record['lap_time_s'], 2) if record['lap_time_s'] else 0.0
        top = fastest_second_mph(record) or record.get('max_speed_mph', 0.0)
        return [record['run'], record['when'], record['mode'], record['result'], record['lap_time_s'],
                record['distance_m'], mph, top, '%d of %d' % (record['checkpoints_passed'], record['checkpoints_total']),
                record.get('barrel_contacts', ''), record.get('lane_line_touches', ''), record.get('pothole_touches', ''),
                record['min_barrel_clearance_m'], record.get('scored_with', 'not scored'),
                record.get('layout_id', 'original'), record.get('route_used', ''),
                'yes' if new_best else 'no', record.get('fail_reason') or '']

    def save_run(self, record):
        """Store a run. Returns (file name, True if it became the new best route)."""
        number = self.run_count() + 1
        record['run'] = number
        name = 'run_%03d_%s_%s.json' % (number, record['mode'], record['result'])
        with open(os.path.join(self.folder, 'runs', name), 'w', encoding='utf-8') as fh:
            json.dump(record, fh)
        new_best = self._consider(record, name)
        if new_best:
            self.keep_lane_memory(record)
        new_file = not os.path.isfile(self.history_path)
        with open(self.history_path, 'a', encoding='utf-8', newline='') as fh:
            w = csv.writer(fh)
            if new_file:
                w.writerow(self.HISTORY_COLUMNS)
            w.writerow(self._history_row(record, new_best))
        return name, new_best

    def rebuild(self):
        """Write history.csv and best_route.json again from the files in runs. Use it after run files
        were copied in by hand, or after the ranking rule changed."""
        folder = os.path.join(self.folder, 'runs')
        files = sorted(fn for fn in os.listdir(folder) if fn.endswith('.json'))
        if os.path.isfile(self.best_path):
            os.replace(self.best_path, self.best_path + '.before_rebuild')
        rows = []
        for number, fn in enumerate(files, 1):
            with open(os.path.join(folder, fn), encoding='utf-8') as fh:
                record = json.load(fh)
            record['run'] = number
            rows.append(self._history_row(record, self._consider(record, fn)))
        with open(self.history_path, 'w', encoding='utf-8', newline='') as fh:
            w = csv.writer(fh)
            w.writerow(self.HISTORY_COLUMNS)
            w.writerows(rows)
        return len(rows)


def route_points(points, step, course=None, overshoot=2.5):
    """The route as closely spaced points (x, y, yaw). If a course is given, the route is carried on past
    the finish line, so the robot crosses the line at speed instead of stopping on it."""
    route = resample(points, step)
    if course is not None and route:
        a, b = course.finish_line
        mid = ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)
        dx, dy = course.travel_direction
        lx, ly = b[0] - a[0], b[1] - a[1]
        width = math.hypot(lx, ly)
        last = route[-1]
        side = ((last[0] - mid[0]) * lx + (last[1] - mid[1]) * ly) / max(1e-9, width)
        side = max(-0.25 * width, min(0.25 * width, side))
        along = (last[0] - mid[0]) * dx + (last[1] - mid[1]) * dy
        if abs(along) < 6.0:        # the route really ends near the line
            yaw = math.atan2(dy, dx)
            d = max(along, 0.0) + step
            while d <= overshoot + 1e-9:
                route.append((mid[0] + side * lx / width + d * dx, mid[1] + side * ly / width + d * dy, yaw))
                d += step
    return route


def goals_from_route(points, spacing, course=None, overshoot=2.5, as_goals=False):
    """Goal poses (x, y, yaw) along a route.

    The first point is dropped (the robot is standing on it). If a course is given, one more goal is
    added beyond the finish line, so the robot crosses the line at speed instead of stopping on it.
    """
    if as_goals:
        goals = []
        for i in range(1, len(points)):
            j = min(i + 1, len(points) - 1)
            k = j - 1
            goals.append((points[i][0], points[i][1],
                          math.atan2(points[j][1] - points[k][1], points[j][0] - points[k][0])))
    else:
        goals = resample(points, spacing)[1:]
    if course is not None and goals:
        a, b = course.finish_line
        mid = ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)
        dx, dy = course.travel_direction
        # keep the route's sideways position at the line, but stay within the middle half of it
        lx, ly = b[0] - a[0], b[1] - a[1]
        width = math.hypot(lx, ly)
        side = ((goals[-1][0] - mid[0]) * lx + (goals[-1][1] - mid[1]) * ly) / max(1e-9, width)
        side = max(-0.25 * width, min(0.25 * width, side))
        gx = mid[0] + side * lx / width + overshoot * dx
        gy = mid[1] + side * ly / width + overshoot * dy
        along_last = (goals[-1][0] - mid[0]) * dx + (goals[-1][1] - mid[1]) * dy
        if along_last < overshoot - 0.5:
            goals.append((gx, gy, math.atan2(dy, dx)))
    return goals


def advance_index(goals, index, xy, reach, look_ahead=3):
    """Index of the goal the robot should head for now.

    Moves on when the robot is within reach of the present goal, or already closer to one of the
    next few goals than to the present one. Never moves backward.
    """
    while index < len(goals) - 1:
        d_here = math.hypot(goals[index][0] - xy[0], goals[index][1] - xy[1])
        if d_here <= reach:
            index += 1
            continue
        better, d_best = None, d_here
        for j in range(index + 1, min(len(goals), index + 1 + look_ahead)):
            d = math.hypot(goals[j][0] - xy[0], goals[j][1] - xy[1])
            if d < d_best:
                better, d_best = j, d
        if better is None:
            break
        index = better
    return index
