import board
from tmc2209.tmc2209_uart import *

pin_en = digitalio.DigitalInOut(board.GP2)
pin_en.direction = digitalio.Direction.OUTPUT
pin_en.value=False

uart = busio.UART(board.GP4, board.GP5, baudrate=115200)

tmc_azm=TMC_uart(uart, mtr_id=0)
tmc_alt=TMC_uart(uart, mtr_id=1)

tmc_azm.set_current(300)
tmc_azm.set_interpolation(True)
tmc_azm.set_spreadcycle(False)
tmc_azm.set_microstepping_resolution(2)
tmc_azm.set_internal_rsense(False)

tmc_azm.read_ioin()
tmc_azm.read_chopconf()
tmc_azm.read_drv_status()
tmc_azm.read_gconf()

print("------")
tmc_alt.read_drv_status()
print("------")

tmc_azm.set_vactual_dur(1000, duration=10)
