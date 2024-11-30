import board
import digitalio
import time

# Pins
dis_pin = digitalio.DigitalInOut(board.GP2)
dis_pin.direction = digitalio.Direction.OUTPUT
dir_pin = digitalio.DigitalInOut(board.GP6)
dir_pin.direction = digitalio.Direction.OUTPUT
step_pin = digitalio.DigitalInOut(board.GP7)
step_pin.direction = digitalio.Direction.OUTPUT

# Anable
dis_pin.value = False

# Direction
dir_pin.value = False

# Lets make 200 steps

for i in range(200):
    step_pin.value = not(step_pin.value)
    time.sleep(0.01)
