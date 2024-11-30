import serial
import time
import curses
from adafruit import adafruit_gps

#--------------------------------------------------------------------------------------
# Main
#--------------------------------------------------------------------------------------

with serial.Serial ("/dev/ttyAMA1", baudrate=9600) as uart:

    stdscr = curses.initscr()
    curses.noecho()
    curses.cbreak()
    
    gps = adafruit_gps.GPS(uart, debug=False)
    # Turn on the basic GGA and RMC info (what you typically want)
    gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")    
    # Set update rate to once a second (1hz) which is what you typically want.
    gps.send_command(b"PMTK220,1000")
    
    try:        
        last_print = time.monotonic()
        while True:
            # Make sure to call gps.update() every loop iteration and at least twice
            # as fast as data comes from the GPS unit (usually every second).
            # This returns a bool that's true if it parsed new data (you can ignore it
            # though if you don't care and instead look at the has_fix property).
            gps.update()
            # Every second print out current location details if there's a fix.
            current = time.monotonic()
            if current - last_print >= 1.0:
                stdscr.clear()
                
                last_print = current
                if not gps.has_fix:
                    # Try again if we don't have a fix yet.
                    stdscr.addstr(0, 0, "Waiting for fix...")
                    continue
                # We have a fix! (gps.has_fix is true)
                # Print out details about the fix like location, date, etc.
                stdscr.addstr(0, 0, "Fix timestamp:        {}/{}/{} {:02}:{:02}:{:02}".format(
                        gps.timestamp_utc.tm_mon,  # Grab parts of the time from the
                        gps.timestamp_utc.tm_mday,  # struct_time object that holds
                        gps.timestamp_utc.tm_year,  # the fix time.  Note you might
                        gps.timestamp_utc.tm_hour,  # not get all data like year, day,
                        gps.timestamp_utc.tm_min,  # month!
                        gps.timestamp_utc.tm_sec,
                    )
                )
                stdscr.addstr(1, 0, "Latitude:             {0:.6f} degrees".format(gps.latitude))
                stdscr.addstr(2, 0, "Longitude:            {0:.6f} degrees".format(gps.longitude))

                # and might not be present.  Check if they're None before trying to use!
                if gps.satellites is not None:
                    stdscr.addstr(4, 0, "Number of satellites: {}".format(gps.satellites))
                if gps.altitude_m is not None:
                    stdscr.addstr(5, 0, "Altitude:             {} meters".format(gps.altitude_m))
                
                stdscr.addstr(8, 0, "Press ^C to exit")
                stdscr.refresh()
                
    except:
        curses.echo()
        curses.nocbreak()
        curses.endwin()
        
        print ('Bye')
