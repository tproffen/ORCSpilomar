import board
from tmc2209.tmc2209_uart import *

uart = busio.UART(board.GP4, board.GP5)
tmc=TMCStepper(uart, mtr_id = 0)

tmc.set_motor_enabled(True)

tmc.set_current(300)
tmc.set_interpolation(True)
tmc.set_spreadcycle(False)
tmc.set_microstepping_resolution(2)
tmc.set_internal_rsense(False)

tmc.read_ioin()
tmc.read_chopconf()
tmc.read_drv_status()
tmc.read_gconf()

print("------")
tmc.mtr_id=1
tmc.read_drv_status()
tmc.mtr_id=0


tmc.set_vactual_dur(500, duration=10)
tmc.set_motor_enabled(False)
