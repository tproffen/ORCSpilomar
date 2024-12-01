import board
import time
from analogio import AnalogIn

analog_in = AnalogIn(board.A0)

while True:
    value=analog_in.value
    print(f"Raw: {value} - Voltage: {4.0*(value * 3.3) / 65536:.2f}V") # for 10K - 3.3K
    time.sleep(0.1)
