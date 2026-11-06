import numpy as np

from imprimis_perception.lane_projection import obstacle_cells, reject_hidden, reject_near_obstacles

CAMERA_XY = np.array([0.198, 0.0175])
CAMERA_HEIGHT = 0.686
GROUND_Z = -0.1227
LIDAR_T = np.array([-0.36, 0.0, 0.93])       # about where the VLP-16 sits in base_link


def wall(x0, x1, y0, y1, height, step=0.05):
    n = int(max(abs(x1 - x0), abs(y1 - y0)) / step) + 1
    return np.column_stack([np.linspace(x0, x1, n), np.linspace(y0, y1, n), np.full(n, height)])


def test_ground_behind_a_wall_is_hidden():
    obstacles = wall(2.0, 2.0, -1.0, 1.0, 0.9)
    kept = reject_hidden(np.array([[1.5, 0.0], [3.0, 0.0], [3.5, 0.4]]), CAMERA_XY, CAMERA_HEIGHT, obstacles)
    assert kept.tolist() == [[1.5, 0.0]]


def test_camera_sees_over_a_low_box():
    obstacles = wall(2.0, 2.0, -0.3, 0.3, 0.2)
    # a 0.2 m box 1.8 m out hides the ground to about 1.8 * 0.686 / 0.486 = 2.5 m from the camera
    kept = reject_hidden(np.array([[2.4, 0.0], [3.5, 0.0]]), CAMERA_XY, CAMERA_HEIGHT, obstacles)
    assert kept.tolist() == [[3.5, 0.0]]


def test_lane_beside_a_wall_is_kept_and_the_foot_of_the_wall_is_not():
    obstacles = wall(0.5, 4.5, 1.6, 1.6, 0.9)
    lane = np.column_stack([np.linspace(1.0, 4.0, 7), np.full(7, 1.3)])
    behind = np.column_stack([np.linspace(2.0, 4.0, 5), np.full(5, 2.2)])
    kept = reject_hidden(np.vstack([lane, behind]), CAMERA_XY, CAMERA_HEIGHT, obstacles)
    assert np.allclose(kept, lane)


def test_nothing_to_hide_behind():
    xy = np.array([[2.0, 0.5], [3.0, -0.5]])
    assert np.array_equal(reject_hidden(xy, CAMERA_XY, CAMERA_HEIGHT, np.zeros((0, 3))), xy)


def test_near_obstacle_radius():
    kept = reject_near_obstacles(np.array([[2.0, 0.0], [2.0, 1.0]]), np.array([[2.1, 0.05]]), radius=0.2)
    assert kept.tolist() == [[2.0, 1.0]]


def corridor_scan(floor_fraction=0.1, seed=0):
    rng = np.random.default_rng(seed)
    n = 4000
    nf = int(n * floor_fraction)
    floor = np.column_stack([rng.uniform(0.5, 5.0, nf), rng.uniform(-1.4, 1.4, nf), np.full(nf, 0.0)])
    nw = (n - nf) // 2
    left = np.column_stack([rng.uniform(0.5, 5.0, nw), np.full(nw, 1.5), rng.uniform(0.0, 2.4, nw)])
    right = np.column_stack([rng.uniform(0.5, 5.0, nw), np.full(nw, -1.5), rng.uniform(0.0, 2.4, nw)])
    ceiling = np.column_stack([rng.uniform(3.0, 5.0, 200), rng.uniform(-1.4, 1.4, 200), np.full(200, 2.5)])
    base = np.vstack([floor, left, right, ceiling]) + np.array([0.0, 0.0, GROUND_Z])
    return base - LIDAR_T, base


def test_corridor_walls_found_when_the_floor_is_scarce():
    points, _ = corridor_scan(floor_fraction=0.1)
    cells = obstacle_cells(points, np.eye(3), LIDAR_T, GROUND_Z)
    assert len(cells) > 50
    assert np.all(np.abs(np.abs(cells[:, 1]) - 1.5) < 0.11), 'only the walls stand up'
    assert cells[:, 2].max() <= 1.0, 'the ceiling is left out'
    assert np.any(cells[:, 1] > 0) and np.any(cells[:, 1] < 0)


def test_corridor_with_plenty_of_floor():
    points, _ = corridor_scan(floor_fraction=0.6)
    cells = obstacle_cells(points, np.eye(3), LIDAR_T, GROUND_Z)
    assert len(cells) > 50
    assert np.all(np.abs(np.abs(cells[:, 1]) - 1.5) < 0.11)


def test_a_wall_is_not_taken_for_the_ground():
    points, base = corridor_scan(floor_fraction=0.05)
    one_side = base[:, 1] > -1.45
    cells = obstacle_cells(points[one_side], np.eye(3), LIDAR_T, GROUND_Z)
    assert np.all(cells[:, 1] > 1.39)


def test_hallway_end_to_end():
    points, _ = corridor_scan(floor_fraction=0.1)
    cells = obstacle_cells(points, np.eye(3), LIDAR_T, GROUND_Z)
    tape = np.column_stack([np.linspace(1.5, 4.0, 6), np.full(6, 1.2)])
    door_frame = np.column_stack([np.linspace(2.5, 4.0, 4), np.full(4, 2.0)])
    kept = reject_hidden(np.vstack([tape, door_frame]), CAMERA_XY, CAMERA_HEIGHT, cells)
    assert np.allclose(kept, tape)
