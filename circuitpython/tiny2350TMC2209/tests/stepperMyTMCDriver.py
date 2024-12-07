import board
from tmc2209.tmc2209_stepper import *

pin_en = digitalio.DigitalInOut(board.GP2)
pin_en.direction = digitalio.Direction.OUTPUT
pin_en.value=False

pin_dir = digitalio.DigitalInOut(board.GP3)
pin_dir.direction = digitalio.Direction.OUTPUT

azm_step = digitalio.DigitalInOut(board.GP29)
azm_step.direction = digitalio.Direction.OUTPUT
alt_step = digitalio.DigitalInOut(board.GP28)
alt_step.direction = digitalio.Direction.OUTPUT

uart = busio.UART(board.GP4, board.GP5, baudrate=115200)

tmc_azm=TMCStepper(uart, mtr_id=0, pin_step=azm_step, pin_dir=pin_dir)
tmc_alt=TMCStepper(uart, mtr_id=1, pin_step=alt_step, pin_dir=pin_dir)

ms=16
motors=[tmc_alt, tmc_azm]
for motor in motors:
    motor.set_current(300)
    motor.set_interpolation(True)
    motor.set_spreadcycle(False)
    motor.set_microstepping_resolution(ms)
    motor.set_internal_rsense(False)
    motor.set_acceleration_fullstep(500)
    motor.set_max_speed_fullstep(200)
    motor.set_speed_fullstep(2000)

    start=time.monotonic_ns()
    motor.run_to_position_steps(ms*400)
    elapsed=(time.monotonic_ns()-start)/1e9
    print (f"Motor full revolution - {elapsed:.2f} sec")
    print("--------------------------------")
