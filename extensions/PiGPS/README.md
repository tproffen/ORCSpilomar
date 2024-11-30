# PiGPS

Adaped the <a href="https://github.com/adafruit/Adafruit_CircuitPython_GPS">Adafruit CircuitPython GPS library</a> to run on the Raspberry Pi and added some examples. Tested with the <a href="https://www.adafruit.com/product/5440">Adafruit Ultimate GPS Breakout</a> board.

This will be running on the <a href="https://www.instructables.com/Pi-lomar-3D-Printed-Working-Miniature-Observatory-/">Pilomar telescope</a> which uses the default uart. The Raspberry Pi 4 allows to run multiple UARTs at the same time. To enable, add these lines to `/boot/config.txt`

```
# Enable UART5 for GPS
dtoverlay=uart5
```

We are using UART5 which uses GPIO pints 12 and 13 which are broken out on the Pilomar PCB :)
