"""Exports the robot's parts from mechanics/freecad/rat.fcstd to sim/assets/*.stl (assembly frame, mm).

    ~/programs/FreeCAD_*.AppImage freecadcmd software/ratatron/sim/export_cad.py

The printed parts (body, chassis, battery_cover, motor_support) are copied from mechanics/stl as they are.
The CAD holds several design iterations: these are the objects of the current robot.
"""
import os
import shutil

import FreeCAD
import Mesh
import MeshPart

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))     # software/ratatron/sim -> the repo
OUT = os.path.join(HERE, 'assets')
PARTS = {
    'motor_l': ['Clone'], 'motor_r': ['Clone001'],
    'rim_l': ['Clone006'], 'tire_l': ['Clone007'], 'rim_r': ['Clone008'], 'tire_r': ['Clone009'],
    'pcb': ['Cut011'],
    'ir_fl': ['Clone031'], 'ir_fr': ['Clone030'],         # GP2Y0A51SK0F facing forward, outside
    'ir_sl': ['Clone032'], 'ir_sr': ['Part__Mirroring'],  # at the nose, angled 15 deg forward
    'buttons': ['Clone048', 'Clone053', 'Clone054'],
    'bolts': ['Chamfer055', 'Chamfer056', 'Chamfer057', 'Chamfer058'],
}

doc = FreeCAD.openDocument(os.path.join(ROOT, 'mechanics', 'freecad', 'rat.fcstd'))
os.makedirs(OUT, exist_ok=True)
for name, objects in PARTS.items():
    mesh = Mesh.Mesh()
    for o in objects:
        mesh.addMesh(MeshPart.meshFromShape(Shape=doc.getObject(o).Shape, LinearDeflection=0.15, AngularDeflection=0.3))
    mesh.write(os.path.join(OUT, name + '.stl'))
for name in ('body', 'chassis', 'battery_cover', 'motor_support'):
    shutil.copy(os.path.join(ROOT, 'mechanics', 'stl', name + '.stl'), OUT)
