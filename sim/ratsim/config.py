"""The robot's constants, read from the firmware's robot_config.h (one source of truth) and the CAD geometry."""
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
FIRMWARE_SRC = os.path.join(ROOT, 'firmware', 'src')
SIM_DIR = os.path.join(ROOT, 'sim')
ASSETS = os.path.join(SIM_DIR, 'assets')


def firmware_constants(path=os.path.join(FIRMWARE_SRC, 'robot_config.h')):
    values = {}
    with open(path) as f:
        for line in f:
            m = re.match(r'#define\s+(\w+)\s+\(?(-?[\d.]+)f?\)?', line)
            if m:
                values[m.group(1)] = float(m.group(2))
    return values


FW = firmware_constants()

# ---- Maze (micromouse rules, mm) --------------------------------------------------------------------------
CELL_MM = FW['CELL_MM']             # 180
WALL_MM = 12.0                      # wall and post thickness
WALL_H_MM = 50.0                    # wall height
CAP_MM = 1.0                        # the red top of every wall and post

# ---- Robot (mm, FreeCAD assembly frame: x right, y forward, z up, origin between the axles at their height) ----
TIRE_RADIUS_MM = 14.3               # tires x 23..36, z -9.3..19.3 (CAD)
AXLE_Z_MM = 5.0                     # wheel centres' height in the CAD frame
GROUND_Z_MM = AXLE_Z_MM - TIRE_RADIUS_MM    # -9.3: the floor in the CAD frame
TIRE_WIDTH_MM = 13.0
WHEEL_X_MM = 29.5                   # tires' centre line (CAD)
# Contact track: a real 90 deg in-place turn takes TICKS_PER_TURN of wheel difference (the tires scrub, the contact
# is nearer the inside edge): the simulated wheels touch the floor this far from the centre, the same 403 ticks.
CONTACT_X_MM = FW['TICKS_PER_TURN'] / FW['WHEEL_TICKS_PER_MM'] / (3.14159265 / 2)
MASS_KG = 0.100                     # assumed (not measured): micromouse of this size with a 2S LiPo
# Centre of mass, estimated from the CAD by parts (2S LiPo ~25 g behind the axle at y -22; motors, wheels and PCB on
# it; the printed body and the sensors ahead): on the axle. 6 mm ahead made every fast curve slide wide.
COM_MM = (0.0, 0.5, 8.0)
SKID_Z_MM = -7.0                    # the chassis' lowest points, front (y 9..26) and back (y -29..-16): 2.3 mm up
# Sensors (Sharp GP2Y0A51SK0F, 27 mm tall bodies in the CAD): the centre of each front face, its direction (deg
# from +x, counter-clockwise): FL/FR outside, facing forward; SL/SR at the nose, angled 15 deg forward.
IR_SENSORS = {          # name: (firmware index, ADC channel, (x, y, z) mm, direction deg)
    'FL': (0, 9, (-27.5, 29.1, 9.0), 90.0),
    'FR': (1, 6, (27.5, 29.1, 9.0), 90.0),
    'SL': (2, 8, (-15.19, 37.32, 9.0), 165.0),
    'SR': (3, 7, (15.19, 37.32, 9.0), 15.0),
}
