import serial
import time
import curses
from adafruit import adafruit_gps

#--------------------------------------------------------------------------------------
# Some helper routines for output
#--------------------------------------------------------------------------------------

def format_dop(dop):
    # https://en.wikipedia.org/wiki/Dilution_of_precision_(navigation)
    if dop > 20:
        msg = "Poor"
    elif dop > 10:
        msg = "Fair"
    elif dop > 5:
        msg = "Moderate"
    elif dop > 2:
        msg = "Good"
    elif dop > 1:
        msg = "Excellent"
    else:
        msg = "Ideal"
    return f"{dop} - {msg}"


talkers = {
    "GA": "Galileo",
    "GB": "BeiDou",
    "GI": "NavIC",
    "GL": "GLONASS",
    "GP": "GPS",
    "GQ": "QZSS",
    "GN": "GNSS",
}

#--------------------------------------------------------------------------------------
# Main
#--------------------------------------------------------------------------------------

with serial.Serial ("/dev/ttyAMA1", baudrate=9600) as uart:

    stdscr = curses.initscr()
    curses.noecho()
    curses.cbreak()
    
    gps = adafruit_gps.GPS(uart, debug=False)

    # Turn on everything (not all of it is parsed!)
    gps.send_command(b"PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0")
    # Set update rate to once a second (1hz) which is what you typically want.
    gps.send_command(b"PMTK220,1000")


    try:        
        last_print = time.monotonic()
        while True:
            # Make sure to call gps.update() every loop iteration and at least twice
            # as fast as data comes from the GPS unit (usually every second).
            # This returns a bool that's true if it parsed new data (you can ignore it
            # though if you don't care and instead look at the has_fix property).
            if not gps.update() or not gps.has_fix:
                time.sleep(0.1)
                continue

            if gps.nmea_sentence[3:6] == "GSA":
                stdscr.clear()
                stdscr.addstr(0, 0, 
                              f"Location: {gps.latitude:.6f}, {gps.longitude:.6f} Altitude: {gps.altitude_m}m")
                stdscr.addstr(2, 0, f"2D Fix: {gps.has_fix}  3D Fix: {gps.has_3d_fix}")
                stdscr.addstr(3, 0, f"  PDOP (Position Dilution of Precision): {format_dop(gps.pdop)}")
                stdscr.addstr(4, 0, f"  HDOP (Horizontal Dilution of Precision): {format_dop(gps.hdop)}")
                stdscr.addstr(5, 0, f"  VDOP (Vertical Dilution of Precision): {format_dop(gps.vdop)}")
                stdscr.addstr(7, 0, "Satellites used for fix:")
                for count, s in enumerate(gps.sat_prns):
                    talker = talkers[s[0:2]]
                    number = s[2:]
                    stdscr.addstr(count+8, 0, f"  {talker}-{number} ")
                    if gps.sats is None:
                        stdscr.addstr(count+8, 15,"- no info")
                    else:
                        try:
                            sat = gps.sats[s]
                            if sat is None:
                                stdscr.addstr(count+8, 15, "- no info")
                            else:
                                stdscr.addstr(count+8, 15, 
                                              f"Elevation:{sat[1]}* Azimuth:{sat[2]}* SNR:{sat[3]}dB")
                        except KeyError:
                            stdscr.addstr(count+8, 15, "- no info")
                            
                stdscr.addstr(12, 0, "Press ^C to exit")
                stdscr.refresh()
    
    except:
        curses.echo()
        curses.nocbreak()
        curses.endwin()
        
        print ('Bye')