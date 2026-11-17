"""The simulated robot: MuJoCo physics, the motors, the encoders and the IR sensors around the firmware.

Each millisecond (the firmware's SysTick): the wheel angles become encoder ticks, the rays of the IR sensors become
ADC counts, the firmware runs 1 ms and its PWM drives the motors through two physics steps of 0.5 ms.
"""
import math
import random

import mujoco
import numpy as np

from . import config as C
from .firmware import Firmware
from .model import MM, WALL_GROUP, scene_xml

KV = {'l': C.FW['MOTOR_KV_L'], 'r': C.FW['MOTOR_KV_R']}    # PWM per mm/s
KS = C.FW['MOTOR_KS_PWM']           # PWM lost to friction while moving
STICTION_PWM = 85.0                 # PWM needed to start a wheel (as test/host/control_sim.c)
TAU_S = C.FW['MOTOR_TAU_S']         # the loaded motor's time constant
TICKS_PER_MM = C.FW['WHEEL_TICKS_PER_MM']
WHEEL_DIFF = C.FW['WHEEL_DIFF']     # the left wheel travels this much less per tick

# The GP2Y0A51SK0F measures every ~16.5 ms and holds its output: the only delay modelled. A further fixed delay
# on the fronts (6, 14 ms) left the front stops farther from the real robot's (front aligned err, median -5.3 mm).
IR_PERIOD_MS = 16
IR_NOISE_MM = 0.8
IR_FAR_MM = 300.0                   # nothing in the beam: the farthest reading (the cubic clamps it)
IR_SPREAD_DEG = 2.5                 # three rays: the beam's spot (posts and wall ends read in between)
BUTTON_MS = 80


class Robot:
    def __init__(self, maze, lib_path, flash_file=None, seed=0, start=(0, 0, 'N')):
        self.maze = maze
        self.model = mujoco.MjModel.from_xml_string(scene_xml(maze))
        self.data = mujoco.MjData(self.model)
        self.rng = random.Random(seed)
        m = self.model
        self.body = m.body('robot').id
        self.free_qpos = m.joint('robot').qposadr[0]
        self.free_dof = m.joint('robot').dofadr[0]
        self.wheel_qpos = {s: m.joint(f'wheel_{s}').qposadr[0] for s in 'lr'}
        self.wheel_dof = {s: m.joint(f'wheel_{s}').dofadr[0] for s in 'lr'}
        self.act = {s: m.actuator(f'motor_{s}').id for s in 'lr'}
        self.sites = {n: m.site(f'ir_{n}').id for n in C.IR_SENSORS}
        self.led_geoms = [m.geom(f'led{i}').id for i in range(1, 7)]
        self.ray_group = np.zeros(6, np.uint8)
        self.ray_group[WALL_GROUP] = 1
        self.ray_geom = np.zeros(1, np.int32)
        self._setup_motors()
        self._ir_offsets()
        self.lib_path, self.flash_file = lib_path, flash_file
        self.fw = None
        self.place(*start)
        self.boot()

    # ---- setup ---------------------------------------------------------------------------------------------
    def _setup_motors(self):
        """Motor = torque b * (w_target - w): the joint damping b, the drive b * w_target. b gives the loaded
        motor the firmware's measured time constant on a straight (each wheel carries half the robot)."""
        r = C.TIRE_RADIUS_MM * MM
        m = self.model
        self.r = r
        for s in 'lr':
            body = m.body(f'wheel_{s}').id
            j_wheel = m.body_inertia[body][0] + m.dof_armature[self.wheel_dof[s]]
            j_eff = 0.5 * C.MASS_KG * r * r + j_wheel
            self.b = j_eff / TAU_S
            m.dof_damping[self.wheel_dof[s]] = self.b
            m.actuator_ctrllimited[self.act[s]] = 0

    def _ir_offsets(self):
        """Reading = beam length + offset: each sensor gives the firmware's reference readings (robot_config.h)
        with the robot centred in a cell (sides) and facing a wall there (fronts)."""
        half = 0.5 * C.FW['LANE_WIDTH_MM']
        front_ref, skew = C.FW['FRONT_WALL_REF_MM'], C.FW['FRONT_SQUARE_OFFSET_MM']
        want = {'SL': C.FW['SIDE_CENTER_L_MM'], 'SR': C.FW['SIDE_CENTER_R_MM'],
                'FL': front_ref + skew / 2, 'FR': front_ref - skew / 2}
        self.ir_offset = {}
        for name, (_, _, (x, y, _), deg) in C.IR_SENSORS.items():
            if name in ('FL', 'FR'):
                length = half - y
            else:
                length = (half - abs(x)) / abs(math.cos(math.radians(deg)))
            self.ir_offset[name] = want[name] - length
        # raw counts for a reading: the inverse of the firmware's own calibration (ir_mm_of), by table
        self.ir_held = {n: IR_FAR_MM for n in C.IR_SENSORS}
        self.ir_phase = {n: self.rng.randrange(IR_PERIOD_MS) for n in C.IR_SENSORS}

    def boot(self, cause='power-on'):
        if self.fw:
            self.fw.close()
        self.fw = Firmware(self.lib_path, self.flash_file, cause)
        self.enc_zero = {s: self.data.qpos[self.wheel_qpos[s]] for s in 'lr'}
        self.tables = {}
        counts = np.arange(4096, dtype=np.float32)
        for name, (idx, _, _, _) in C.IR_SENSORS.items():
            self.tables[name] = np.array([self.fw.ir_mm_of(idx, float(c)) for c in counts])   # decreasing
        self.buttons = []           # queued presses: ('start' | 'select')
        self.button_until = 0
        self.button_now = (0, 0)

    def place(self, x, y, heading='N'):
        """Robot standing at the centre of cell (x, y) facing heading (N/E/S/W), at rest."""
        yaw = {'N': 0.0, 'W': 90.0, 'S': 180.0, 'E': -90.0}[heading]
        q = self.data.qpos
        a = self.free_qpos
        q[a:a + 3] = ((x + 0.5) * C.CELL_MM * MM, (y + 0.5) * C.CELL_MM * MM, -C.GROUND_Z_MM * MM + 0.0002)
        h = math.radians(yaw) / 2
        q[a + 3:a + 7] = (math.cos(h), 0, 0, math.sin(h))
        self.data.qvel[:] = 0
        mujoco.mj_forward(self.model, self.data)

    # ---- sensors -------------------------------------------------------------------------------------------------
    def _ray(self, origin, direction):
        d = mujoco.mj_ray(self.model, self.data, origin, direction, self.ray_group, 1, self.body, self.ray_geom)
        return d / MM if d >= 0 else IR_FAR_MM

    def ir_true(self, names=tuple(C.IR_SENSORS)):
        """The sensors' readings now (mm, before the hold and the noise)."""
        out = {}
        R = self.data.site_xmat
        for name in names:
            deg = C.IR_SENSORS[name][3]
            sid = self.sites[name]
            p = self.data.site_xpos[sid]
            rot = R[sid].reshape(3, 3)
            inv = 0.0
            for spread in (-IR_SPREAD_DEG, 0.0, IR_SPREAD_DEG):
                a = math.radians(deg + spread)
                v = rot @ np.array([math.cos(a), math.sin(a), 0.0])
                d = min(self._ray(p, v) + self.ir_offset[name], IR_FAR_MM)
                inv += 1.0 / max(d, 1.0)
            out[name] = 3.0 / inv
        return out

    def _adc(self, t):
        """{channel: counts} of the sensors that measured this ms (the others hold their output)."""
        due = [n for n in C.IR_SENSORS if (t + self.ir_phase[n]) % IR_PERIOD_MS == 0]
        adc = {}
        for name, reading in self.ir_true(due).items():
            self.ir_held[name] = reading + self.rng.gauss(0.0, IR_NOISE_MM)
            table = self.tables[name]
            mm = self.ir_held[name]
            # table decreasing: the counts whose reading is mm, interpolated
            i = int(np.searchsorted(-table, -mm))
            if i <= 0:
                counts = 0.0
            elif i >= 4096:
                counts = 4095.0
            else:
                a, b = table[i - 1], table[i]
                counts = i - 1 + (a - mm) / (a - b) if a != b else float(i)
            adc[C.IR_SENSORS[name][1]] = counts
        return adc

    def encoders(self):
        q = self.data.qpos
        mm_l = (q[self.wheel_qpos['l']] - self.enc_zero['l']) * self.r / MM
        mm_r = (q[self.wheel_qpos['r']] - self.enc_zero['r']) * self.r / MM
        return math.floor(mm_l * TICKS_PER_MM / (1.0 + WHEEL_DIFF)), math.floor(mm_r * TICKS_PER_MM)

    # ---- buttons, console -------------------------------------------------------------------------------------------
    def press(self, button):
        self.buttons.append(button)

    def _buttons(self, t):
        if t < self.button_until:
            return self.button_now
        if self.button_now != (0, 0):
            self.button_now = (0, 0)
            self.button_until = t + BUTTON_MS
            return self.button_now
        if self.buttons:
            b = self.buttons.pop(0)
            self.button_now = (1, 0) if b == 'start' else (0, 1)
            self.button_until = t + BUTTON_MS
        return self.button_now

    # ---- one millisecond ----------------------------------------------------------------------------------------------
    def _drive(self, s, pwm):
        """The motor's drive torque for a PWM: the speed it would settle at, past the friction."""
        w = self.data.qvel[self.wheel_dof[s]]
        u = float(pwm)
        if abs(w) * self.r / MM < 1.0 and abs(u) <= STICTION_PWM:
            u_eff = 0.0
            self.data.qvel[self.wheel_dof[s]] = 0.0     # held by the gearbox's static friction
        else:
            u_eff = u - math.copysign(KS, w if abs(w) > 1e-3 else u)
        v_target = u_eff / KV[s] * MM       # m/s
        self.data.ctrl[self.act[s]] = self.b * v_target / self.r

    def step(self):
        t = self.fw.time_ms()
        enc_l, enc_r = self.encoders()
        pwm_l, pwm_r = self.fw.step(enc_l, enc_r, self._adc(t), self._buttons(t))
        self._drive('l', pwm_l)
        self._drive('r', pwm_r)
        mujoco.mj_step(self.model, self.data, nstep=2)
        leds = self.fw.leds()
        for i, g in enumerate(self.led_geoms):
            on = leds >> (5 - i) & 1
            self.model.geom_rgba[g] = (1, 0.1, 0.05, 1) if on else (0.25, 0.05, 0.05, 1)
        return self.fw.state

    def pose(self):
        """(x mm, y mm, heading deg: 0 north, positive to the left)."""
        q = self.data.qpos[self.free_qpos:self.free_qpos + 7]
        w, z = q[3], q[6]
        x_, y_ = q[4], q[5]
        yaw = math.degrees(math.atan2(2 * (w * z + x_ * y_), 1 - 2 * (y_ * y_ + z * z)))
        return q[0] / MM, q[1] / MM, yaw

    def crashed(self):
        """The wall or post the hull touches (its geom name), else None."""
        m, d = self.model, self.data
        hull = m.geom('hull').id
        for i in range(d.ncon):
            c = d.contact[i]
            other = c.geom2 if c.geom1 == hull else c.geom1
            if hull in (c.geom1, c.geom2) and m.geom_group[other] == WALL_GROUP:
                return m.geom(other).name or 'wall'
        return None
