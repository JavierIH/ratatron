"""The robot firmware (sim/fw: the real application code on a simulated chip) loaded through ctypes."""
import ctypes as C
import os
import shutil
import subprocess
import tempfile

from .config import SIM_DIR

FW_DIR = os.path.join(SIM_DIR, 'fw')
STATE_NAMES = {0: 'running', 1: 'reset', 2: 'halted', 3: 'fault'}


def build(practice=False):
    """make the library if a source changed; returns its path."""
    args = ['make', '-s', '-C', FW_DIR] + (['PRACTICE=1'] if practice else [])
    subprocess.run(args, check=True)
    return os.path.join(FW_DIR, 'build', 'libratfw_practice.so' if practice else 'libratfw.so')


class Firmware:
    """One boot of the firmware. A reset (RESET, NVIC_SystemReset) needs a new instance: every boot loads its own
    copy of the library, so the static variables start from zero as on the chip."""

    def __init__(self, lib_path, flash_file=None, reset_cause='power-on'):
        self._tmp = tempfile.NamedTemporaryFile(prefix='ratfw_', suffix='.so', delete=False)
        self._tmp.close()
        shutil.copyfile(lib_path, self._tmp.name)
        lib = self.lib = C.CDLL(self._tmp.name)
        lib.sim_step.restype = C.c_uint8
        lib.sim_time_ms.restype = C.c_uint32
        lib.sim_leds.restype = C.c_uint8
        lib.sim_set_adc.argtypes = [C.c_uint8, C.c_float]
        lib.sim_set_encoders.argtypes = [C.c_int32, C.c_int32]
        lib.sim_set_buttons.argtypes = [C.c_uint8, C.c_uint8]
        lib.sim_get_pwm.argtypes = [C.POINTER(C.c_int16), C.POINTER(C.c_int16)]
        lib.sim_uart_rx.argtypes = [C.c_char_p]
        lib.sim_uart_tx.argtypes = [C.c_char_p, C.c_uint32]
        lib.sim_uart_tx.restype = C.c_uint32
        lib.sim_flash_load.argtypes = [C.c_char_p, C.c_uint32]
        lib.sim_flash_image.restype = C.POINTER(C.c_uint8)
        lib.sim_flash_writes.restype = C.c_uint32
        lib.sim_set_reset_cause.argtypes = [C.c_char_p]
        lib.ir_mm_of.argtypes = [C.c_int, C.c_float]
        lib.ir_mm_of.restype = C.c_float
        self.flash_file = flash_file
        self._flash_writes = 0
        self._cause = C.c_char_p(reset_cause.encode())     # kept alive: the firmware keeps the pointer
        lib.sim_set_reset_cause(self._cause)
        if flash_file and os.path.exists(flash_file):
            data = open(flash_file, 'rb').read()
            lib.sim_flash_load(data, len(data))
        self._pl, self._pr = C.c_int16(), C.c_int16()
        self._buf = C.create_string_buffer(1 << 16)
        self.state = 0
        if not lib.sim_boot():
            raise RuntimeError('sim_boot failed')

    def close(self):
        try:
            os.unlink(self._tmp.name)
        except OSError:
            pass

    def ir_mm_of(self, sensor, counts):
        return self.lib.ir_mm_of(sensor, counts)

    def step(self, enc_l, enc_r, adc, buttons):
        """adc: {channel: counts}; buttons: (start, select). One ms; returns (pwm_l, pwm_r)."""
        lib = self.lib
        lib.sim_set_encoders(enc_l, enc_r)
        for ch, counts in adc.items():
            lib.sim_set_adc(ch, counts)
        lib.sim_set_buttons(*buttons)
        self.state = lib.sim_step()
        lib.sim_get_pwm(C.byref(self._pl), C.byref(self._pr))
        writes = lib.sim_flash_writes()
        if writes != self._flash_writes and self.flash_file:
            self._flash_writes = writes
            img = lib.sim_flash_image()
            with open(self.flash_file, 'wb') as f:
                f.write(bytes(img[i] for i in range(2048)))
        return self._pl.value, self._pr.value

    def leds(self):
        return self.lib.sim_leds()

    def time_ms(self):
        return self.lib.sim_time_ms()

    def send(self, line):
        self.lib.sim_uart_rx(line.encode()[:63])

    def received(self):
        n = self.lib.sim_uart_tx(self._buf, len(self._buf))
        return self._buf.raw[:n].decode(errors='replace')
