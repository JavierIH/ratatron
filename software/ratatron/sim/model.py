"""The MuJoCo scene: a micromouse maze (black floor, white walls, red tops) and the robot built from the CAD."""
import json
import os

from . import config as C

MM = 0.001
WALL_GROUP = 1          # geom group of the walls and posts: the only thing the IR rays see
ROBOT_GROUP = 2

# Visual meshes exported from mechanics/freecad/ratatron.fcstd (sim/export_cad.py), in the assembly frame:
# one per colour and body (robot, wheel_l, wheel_r).
with open(os.path.join(C.ASSETS, 'parts.json')) as _fp:
    PARTS = json.load(_fp)
LED_X_MM = [-18.5, -13.5, -8.5, 8.5, 13.5, 18.5]    # LEDs 1-3 left, 4-6 right (the CAD's)


def _f(*v):
    return ' '.join(f'{x:.6g}' for x in v)


def maze_xml(maze):
    cell, t, h, cap = C.CELL_MM * MM, C.WALL_MM * MM, C.WALL_H_MM * MM, C.CAP_MM * MM
    n = maze.size
    geoms = []

    sw, sd = C.SLOT_W_MM * MM, C.SLOT_D_MM * MM

    def block(name, cx, cy, sx, sy, kind='wall'):
        geoms.append(f'<geom name="{name}" class="{kind}" pos="{_f(cx, cy, (h - cap) / 2)}" '
                     f'size="{_f(sx / 2, sy / 2, (h - cap) / 2)}"/>')
        geoms.append(f'<geom class="{kind}_top" pos="{_f(cx, cy, h - cap / 2)}" '
                     f'size="{_f(sx / 2, sy / 2, cap / 2)}"/>')
    # Posts: a slot in every face, so a post is a core inside the slots' bottoms and four corner blocks beside them.
    core, corner = t - 2 * sd, (t - sw) / 2
    q = (t + sw) / 4                # the corner blocks' centres from the post's centre
    for px in range(n + 1):
        for py in range(n + 1):
            cx, cy = px * cell, py * cell
            block(f'post_{px}_{py}', cx, cy, core, core, 'post')
            for i, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
                block(f'post_{px}_{py}_{i}', cx + sx * q, cy + sy * q, corner, corner, 'post')
    # Walls: from face to face of the posts, a tongue (slot wide, slot deep) at each end in the slot.
    for x, y, kind in maze.segments():
        if kind == 'h':
            cx, cy, along = (x + 0.5) * cell, y * cell, (1, 0)
        else:
            cx, cy, along = x * cell, (y + 0.5) * cell, (0, 1)
        long_, end = cell - t, (cell - t + sd) / 2
        block(f'wall_{kind}_{x}_{y}', cx, cy, *((long_, t) if kind == 'h' else (t, long_)))
        tongue = (sd, sw) if kind == 'h' else (sw, sd)
        for s in (1, -1):
            block(f'wall_{kind}_{x}_{y}_{"ab"[s < 0]}', cx + s * along[0] * end, cy + s * along[1] * end, *tongue)
    side = n * cell
    floor = (f'<geom name="floor" type="plane" pos="{_f(side / 2, side / 2, 0)}" size="{_f(side / 2 + 0.3, side / 2 + 0.3, 0.05)}" '
             f'material="floor" condim="3" friction="0.05 0.001 0.0001"/>')
    return floor + '\n'.join(geoms)


def robot_xml():
    z0 = -C.GROUND_Z_MM * MM        # the CAD frame's height above the floor
    r = C.TIRE_RADIUS_MM * MM     # contact: a 2 mm band of the tire at CONTACT_X_MM (the 13 mm cylinder touched on its edges)
    meshes = ''.join(f'<geom class="visual" mesh="{p["mesh"]}" rgba="{_f(*p["rgba"])}"/>'
                     for p in PARTS if p['body'] == 'robot')
    leds = ''.join(f'<geom name="led{i + 1}" class="visual" type="box" pos="{_f(x * MM, -26.5 * MM, 18.5 * MM)}" '
                   f'size="{_f(1.5 * MM, 1.0 * MM, 1.0 * MM)}" rgba="0.25 0.05 0.05 1"/>' for i, x in enumerate(LED_X_MM))
    sites = ''.join(f'<site name="ir_{name}" pos="{_f(*(v * MM for v in pos))}" size="0.0015" group="4" rgba="1 0 0 0.6"/>'
                    for name, (_, _, pos, _) in C.IR_SENSORS.items())
    wheels = ''
    for side, sign in (('l', -1), ('r', 1)):
        cx = sign * C.CONTACT_X_MM * MM
        vis = ''.join(f'<geom class="visual" mesh="{p["mesh"]}" pos="{_f(-cx, 0, -C.AXLE_Z_MM * MM)}" rgba="{_f(*p["rgba"])}"/>'
                      for p in PARTS if p['body'] == f'wheel_{side}')
        wheels += f'''
      <body name="wheel_{side}" pos="{_f(cx, 0, C.AXLE_Z_MM * MM)}">
        <joint name="wheel_{side}" type="hinge" axis="-1 0 0" armature="2e-6"/>
        <geom name="tire_{side}" class="tire" type="cylinder" size="{_f(r, 0.001)}" euler="0 90 0" mass="0.004"/>
        {vis}
      </body>'''
    com = _f(*(v * MM for v in C.COM_MM))
    return f'''
    <body name="robot" pos="{_f(0.5 * C.CELL_MM * MM, 0.5 * C.CELL_MM * MM, z0)}">
      <freejoint name="robot"/>
      <inertial pos="{com}" mass="{C.MASS_KG - 0.008:.4f}" diaginertia="5e-5 4e-5 7e-5"/>
      <geom name="hull" class="hull" type="box" pos="{_f(0, 8 * MM, 15 * MM)}" size="{_f(36 * MM, 37 * MM, 20 * MM)}"/>
      <geom name="skid_front" class="skid" type="sphere" pos="{_f(0, 18 * MM, (C.SKID_Z_MM + 2.5) * MM)}" size="0.0025"/>
      <geom name="skid_rear" class="skid" type="sphere" pos="{_f(0, -22 * MM, (C.SKID_Z_MM + 2.5) * MM)}" size="0.0025"/>
      {meshes}{leds}{sites}{wheels}
    </body>'''


def scene_xml(maze):
    mesh_assets = ''.join(f'<mesh name="{p["mesh"]}" file="{p["mesh"]}.stl" scale="0.001 0.001 0.001" inertia="shell"/>'
                          for p in PARTS)
    centre = maze.size * C.CELL_MM * MM / 2
    return f'''<mujoco model="ratatron_maze">
  <compiler meshdir="{C.ASSETS}" angle="degree" inertiafromgeom="auto"/>
  <option timestep="0.0005" integrator="implicitfast" cone="elliptic" impratio="10" noslip_iterations="4"/>
  <visual>
    <global offwidth="1280" offheight="960" azimuth="90" elevation="-55"/>
    <quality shadowsize="4096"/>
    <headlight ambient="0.35 0.35 0.35" diffuse="0.5 0.5 0.5"/>
  </visual>
  <default>
    <default class="wall"><geom type="box" group="{WALL_GROUP}" rgba="0.95 0.95 0.95 1" contype="1" conaffinity="2"/></default>
    <default class="wall_top"><geom type="box" group="0" rgba="0.85 0.05 0.05 1" contype="0" conaffinity="0"/></default>
    <default class="post"><geom type="box" group="{WALL_GROUP}" rgba="0.72 0.72 0.75 1" contype="1" conaffinity="2"/></default>
    <default class="post_top"><geom type="box" group="0" rgba="0.45 0.02 0.02 1" contype="0" conaffinity="0"/></default>
    <default class="visual"><geom type="mesh" group="{ROBOT_GROUP}" contype="0" conaffinity="0" mass="0"/></default>
    <default class="hull"><geom group="3" contype="2" conaffinity="1" rgba="0 0 1 0.1" mass="0" friction="0.3"/></default>
    <default class="skid"><geom group="3" contype="2" conaffinity="1" rgba="0 0 1 0.3" mass="0" friction="0.05 0.001 0.0001"/></default>
    <default class="tire"><geom group="3" contype="2" conaffinity="1" rgba="0 0 0 0.3" friction="0.6 0.005 0.0001" condim="3" solref="0.004 1"/></default>
  </default>
  <asset>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.25 0.27 0.3" rgb2="0.05 0.05 0.06" width="256" height="256"/>
    <material name="floor" rgba="0.02 0.02 0.02 1" specular="0.1" shininess="0.1"/>
    {mesh_assets}
  </asset>
  <worldbody>
    <light pos="{_f(centre, centre, 3)}" dir="0 0 -1" diffuse="0.7 0.7 0.7" castshadow="true"/>
    <camera name="top" pos="{_f(centre, centre, 4.2)}" xyaxes="1 0 0 0 1 0"/>
    {maze_xml(maze)}
    {robot_xml()}
  </worldbody>
  <actuator>
    <motor name="motor_l" joint="wheel_l"/>
    <motor name="motor_r" joint="wheel_r"/>
  </actuator>
</mujoco>'''
