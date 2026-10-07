"""Read the IGVC course out of the Blender map. Runs inside Blender, without a window:

    blender --background "<map>.blend" --python export_course.py -- <out.json> [<folder for the meshes>]

What is read, by object name:
    barrel...        every barrel in the scene (its color comes from its mesh, barrel_orange and so on).
                     A barrel that is hidden, or sits in the "Barrel prototypes (export)" collection, is left out.
    ramp             position, turn about the vertical axis, and size
    lane_tape...     the lane lines (each is a ribbon; its middle line is taken)
    pothole...       position and size
    waypoint_...     the waypoints of No Man's Land, in name order
Nothing in the .blend file is changed.
"""
import hashlib
import json
import math
import os
import re
import sys

import bpy

argv = sys.argv[sys.argv.index('--') + 1:]
out_json = argv[0]
mesh_dir = argv[1] if len(argv) > 1 else ''
PROTOTYPES = 'Barrel prototypes (export)'


def world_xy(ob):
    t = ob.matrix_world.translation
    return [round(t.x, 3), round(t.y, 3)]


def visible(ob):
    return not (ob.hide_viewport or ob.hide_render or ob.hide_get())


def in_collection(ob, name):
    return any(c.name == name for c in ob.users_collection)


course = {'source': os.path.basename(bpy.data.filepath), 'notes': []}

# ---------------------------------------------------------------- barrels
barrels, used = [], set()
for ob in sorted(bpy.data.objects, key=lambda o: o.name):
    if ob.type != 'MESH' or not ob.data.name.startswith('barrel_') or in_collection(ob, PROTOTYPES) or not visible(ob):
        continue
    color = ob.data.name.split('.')[0].split('_', 1)[1]
    name = re.sub(r'[^A-Za-z0-9_]', '_', ob.name)
    while name in used:
        name += '_b'
    used.add(name)
    barrels.append({'name': name, 'color': color, 'xy': world_xy(ob)})
    if abs(ob.matrix_world.translation.z) > 0.05:
        course['notes'].append('%s is %.2f m off the ground in Blender; it stands on the ground in the simulator.' % (ob.name, ob.matrix_world.translation.z))
course['barrels'] = barrels
proto = next((o for o in bpy.data.objects if o.type == 'MESH' and o.data.name.startswith('barrel_')), None)
dims = proto.data and proto.dimensions if proto else None
course['barrel'] = {'height': round(float(dims.z) / (proto.scale.z or 1.0), 5), 'diameter': round(float(dims.x) / (proto.scale.x or 1.0), 5)} if proto else {'height': 1.00838, 'diameter': 0.5969}

# ---------------------------------------------------------------- ramp
ramp = next((o for o in bpy.data.objects if o.type == 'MESH' and o.name.lower().startswith('ramp') and visible(o)), None)
if ramp is not None:
    d = ramp.dimensions
    course['ramp'] = {'center': world_xy(ramp), 'yaw': round(ramp.matrix_world.to_euler().z, 4), 'length': round(d.x, 3),
                      'width': round(d.y, 3), 'rise': round(d.z, 3), 'grade': round(d.z / (d.x / 2.0), 4) if d.x > 0 else 0.0}
else:
    course['ramp'] = None

# ---------------------------------------------------------------- lane lines, potholes, waypoints
lanes = []
for ob in sorted((o for o in bpy.data.objects if o.type == 'MESH' and o.name.startswith('lane_tape') and visible(o)), key=lambda o: o.name):
    co = [ob.matrix_world @ v.co for v in ob.data.vertices]
    line = [[round((co[i].x + co[i + 1].x) / 2, 3), round((co[i].y + co[i + 1].y) / 2, 3)] for i in range(0, len(co) - 1, 2)]
    lanes.append(line)
course['lane_lines'] = lanes
course['tape_width'] = 0.1016
potholes = []
for ob in sorted((o for o in bpy.data.objects if o.type == 'MESH' and o.name.startswith('pothole') and visible(o)), key=lambda o: o.name):
    co = [ob.matrix_world @ v.co for v in ob.data.vertices]
    cx = sum(c.x for c in co) / len(co)
    cy = sum(c.y for c in co) / len(co)
    potholes.append({'xy': [round(cx, 3), round(cy, 3)], 'diameter': round(max(ob.dimensions.x, ob.dimensions.y), 4)})
course['potholes'] = potholes
course['waypoints'] = [world_xy(o) for o in sorted((o for o in bpy.data.objects if o.type == 'EMPTY' and o.name.startswith('waypoint_')),
                                                  key=lambda o: o.name)]
xs = [p[0] for line in lanes for p in line] or [0.0]
ys = [p[1] for line in lanes for p in line] or [0.0]
course['extent'] = {'x': [min(xs), max(xs)], 'y': [min(ys), max(ys)]}

# A fingerprint of everything that is drawn on the ground. When it changes, the ground mesh is exported again.
h = hashlib.sha1()
for ob in sorted((o for o in bpy.data.objects if o.type == 'MESH' and (o.name.startswith(('lane_tape', 'pothole', 'asphalt')))), key=lambda o: o.name):
    h.update(ob.name.encode())
    for v in ob.data.vertices:
        w = ob.matrix_world @ v.co
        h.update(('%.3f,%.3f,%.3f;' % (w.x, w.y, w.z)).encode())
course['surface_fingerprint'] = h.hexdigest()[:16]

# ---------------------------------------------------------------- meshes for the simulator
if mesh_dir:
    def export(objs, path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        bpy.ops.object.select_all(action='DESELECT')
        for o in objs:
            o.select_set(True)
        bpy.context.view_layer.objects.active = objs[0]
        bpy.ops.wm.obj_export(filepath=path, export_selected_objects=True, forward_axis='Y', up_axis='Z',
                              export_materials=True, export_triangulated_mesh=True, path_mode='STRIP',
                              export_normals=True, export_uv=True)

    surface = [o for o in bpy.data.objects if o.type == 'MESH' and o.name.startswith(('asphalt', 'lane_tape', 'pothole')) and visible(o)]
    if surface:
        export(sorted(surface, key=lambda o: o.name), os.path.join(mesh_dir, 'igvc2027_course', 'meshes', 'course.obj'))
    if ramp is not None:   # exported lying at the origin; the simulator places and turns it
        keep = ramp.matrix_world.copy()
        loc, rot = ramp.location.copy(), ramp.rotation_euler.copy()
        ramp.location = (0.0, 0.0, 0.0)
        ramp.rotation_euler = (0.0, 0.0, 0.0)
        bpy.context.view_layer.update()
        export([ramp], os.path.join(mesh_dir, 'igvc2027_ramp', 'meshes', 'ramp.obj'))
        ramp.location, ramp.rotation_euler = loc, rot
        bpy.context.view_layer.update()

with open(out_json, 'w', encoding='utf-8') as f:
    json.dump(course, f)
print('COURSE EXPORTED: %d barrels, %d lane lines, ramp %s, %d waypoints -> %s'
      % (len(barrels), len(lanes), 'yes' if ramp is not None else 'no', len(course['waypoints']), out_json))
