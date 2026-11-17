"""Exports the robot from mechanics/freecad/ratatron.fcstd to sim/assets (assembly frame, mm).

    ~/programs/FreeCAD_*.AppImage freecadcmd software/ratatron/sim/export_cad.py

Exports what the project shows: every solid visible with its containers, in FreeCAD's per-face colours.
The faces of one colour become one STL per body (robot, wheel_l, wheel_r: the parts of the `wheels` group,
by side); parts.json lists them for model.py.
"""
import json
import os
import struct
import xml.etree.ElementTree as ET
import zipfile

import FreeCAD
import MeshPart
import Part

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))     # software/ratatron/sim -> the repo
FCSTD = os.path.join(ROOT, 'mechanics', 'freecad', 'ratatron.fcstd')
OUT = os.path.join(HERE, 'assets')
WHEELS_GROUP = 'wheels'


def appearances():
    """Object name -> per-face colours [(r, g, b, a)] from the ShapeAppearance lists of GuiDocument.xml."""
    z = zipfile.ZipFile(FCSTD)
    out = {}
    for vp in ET.fromstring(z.read('GuiDocument.xml')).iter('ViewProvider'):
        for p in vp.iter('Property'):
            if p.get('name') == 'ShapeAppearance':
                d = z.read(p.find('MaterialList').get('file'))
                n = struct.unpack('<I', d[:4])[0]       # 24 bytes each: ambient, diffuse, specular, emissive, shininess, transparency
                out[vp.get('name')] = [(d[11 + 24 * i] / 255, d[10 + 24 * i] / 255, d[9 + 24 * i] / 255,
                                        1 - struct.unpack('<f', d[24 + 24 * i:28 + 24 * i])[0]) for i in range(n)]
    return out


def shown(o):
    """Visible, and so are its containers (groups, App::Part, Body)."""
    while o is not None:
        if not o.Visibility:
            return False
        o = o.getParentGeoFeatureGroup() or o.getParentGroup()
    return True


doc = FreeCAD.openDocument(FCSTD)
colours = appearances()
faces = {}                                       # (body, rgba) -> [faces]
for o in doc.Objects:
    if o.isDerivedFrom('App::DocumentObjectGroup') or not hasattr(o, 'Shape') or not shown(o):
        continue
    s = o.Shape
    if s.isNull() or s.Volume < 1e-3:
        continue
    group = o.getParentGroup()
    body = ('wheel_l' if s.BoundBox.Center.x < 0 else 'wheel_r') if group and group.Label == WHEELS_GROUP else 'robot'
    c = colours.get(o.Name) or [(0.8, 0.8, 0.8, 1)]
    for i, f in enumerate(s.Faces):
        rgba = tuple(round(v, 3) for v in (c[i] if len(c) == len(s.Faces) else c[0]))
        faces.setdefault((body, rgba), []).append(f)

os.makedirs(OUT, exist_ok=True)
for f in os.listdir(OUT):
    if f.endswith('.stl'):
        os.remove(os.path.join(OUT, f))
parts = []
for n, ((body, rgba), fs) in enumerate(sorted(faces.items())):
    name = f'{body}_{n:02d}'
    mesh = MeshPart.meshFromShape(Shape=Part.makeCompound(fs), LinearDeflection=0.1, AngularDeflection=0.3)
    mesh.write(os.path.join(OUT, name + '.stl'))
    parts.append({'mesh': name, 'body': body, 'rgba': list(rgba)})
with open(os.path.join(OUT, 'parts.json'), 'w') as fp:
    json.dump(parts, fp, indent=1)
print(f'{len(parts)} meshes in {OUT}')
