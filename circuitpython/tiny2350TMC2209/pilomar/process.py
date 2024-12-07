# Based on https://github.com/Short-bus/pilomar
# Simplified for use with TINY2350

from pilomar.helpers import *

#-----------------------------------------------------------------------------------------------
class process():
    def __init__(self, LogFile, ExceptionCounter, StatusLed, RPi, Session, Clock, Motors):
        self.LogFile = LogFile
        self.ExceptionCounter = ExceptionCounter
        self.StatusLed = StatusLed
        self.RPi = RPi
        self.Session = Session
        self.Clock = Clock
        self.Motors = Motors
        
    def ProcessInput(self, line):
        lineitems = line.split(' ')
        if lineitems[0] == 'exit':
            print('exit cmd received.')
            self.LogFile.Log('exit cmd received.')
            self.Session.Quit = True
        elif lineitems[0] == 'stop': # Immediately stop motion.
            for i in self.Motors:
                i.Stop()
            self.Session.MovePermission()
        elif line.startswith('#'): pass # Ignore comments.
        elif line.startswith('rpi started'):
            self.RPi.Write('acknowledged rpi started')
            for i in self.Motors:
                i.Reset()
            self.Session.MovePermission() # Decide if we have valid trajectories and configuration in every motor. OK to move if we do!
        elif lineitems[0] == 'reset':
            for i in self.Motors:
                i.Reset() # Reset motor status.
            self.RPi.Reset() # Flush output buffers.
            self.RPi.Write('acknowledged reset') # Confirm reset performed.
        elif lineitems[0] == 'sendstatus': # Turn off status messages for motors. Useful when downloading a batch of trajectories, so no conflicting requests exchanged.
            self.RPi.Write('# ' + line)
            for i in self.Motors:
                i.SendStatus = StringToBool(lineitems[2])
        elif line.startswith('set rgb'): # A direct command to set a specific RGB LED color. (Debug support)
            self.StatusLed.SetRGB(line) # Set the RGB color regardless of LED status.
            self.RPi.Write("# Acknowledge set rgb :" + str(line)) # Echo the command back.
        elif line.startswith('raise exception'): # Generate example exception to prove status LED workds as expected.
            self.ExceptionCounter.Raise()
            self.RPi.Write('# Raised artificial exception for testing.')
        elif lineitems[0] == 'tune':
            for i in self.Motors:
                if i.MotorName == lineitems[2]: i.TunePosition(int(lineitems[3]))
        elif line.startswith('rpi version'):
            self.CheckVersionCompatibility(lineitems[3])
        elif line.startswith('clear trajectory'):
            self.RPi.Write('cleared trajectory')
            for i in self.Motors:
                i.Trajectory.Clear()
            self.Session.MovePermission() # Decide if we have valid trajectories and configuration in every motor. OK to move if we do!
        elif line.startswith('configure motor'):
            for i in self.Motors:
                if i.MotorName == lineitems[3]:
                    i.ConfigureMotor(line) # Load configuration.
                    i.SendMotorStatus(immediate=True,codes='cfg') # Immediately respond with lates motor status.
            self.Session.MovePermission() # Decide if we have valid trajectories and configuration in every motor. OK to move if we do!
        elif line.startswith == 'report motor': # RPi has requested the motor configurations to be reported back.
            for i in self.Motors:
                i.ReportMotorConfig()
        elif lineitems[0] == 'trajectory':
            for i in self.Motors:
                if i.MotorName == lineitems[2]:
                    i.AddTrajectoryPoint(line)
            self.Session.MovePermission() # Decide if we have valid trajectories and configuration in every motor. OK to move if we do!
        elif lineitems[0] == 'goto':
            for i in self.Motors:
                if i.MotorName == lineitems[2]:
                    i.GoToAngle(float(lineitems[3]))
        elif line.startswith('set time'):
            self.Clock.SetTimeFromString(lineitems[2])
            self.Session.MovePermission() # Decide if we have valid trajectories and configuration in every motor. OK to move if we do!
        elif line.startswith('leds off'): # Go to stealth mode, turn LEDs off.
            self.StatusLed.Disable() # Disable the onboard status LED.
        elif line.startswith('leds on'): # Enable the LEDs to show processing.
            self.StatusLed.Enable() # Enable the onboard status LED.
        elif lineitems[0] == 'pin': # Direct GPIO pin command.
            self.PinCommand(line) # Execute the pin command.
        else:
            self.RPi.Write('error: unrecognised RPi command: ' + line)

    #-----------------------------------------------------------------------------------------------
    def CheckVersionCompatibility(self, rpiversion):
        """ The Raspberry Pi has sent the version number for pilomar.py
            Check that it's compatible with this code.py program.
            This issues a log file warning. It will not terminate the program. """
        compversion = rpiversion[:rpiversion.rindex('.')]
        self.LogFile.Log('Pi Pilomar version is ',rpiversion)

    #-----------------------------------------------------------------------------------------------
    def PinCommand(self, line):
        """ Receive a direct command for a GPIO pin and execute it."""
        
        lineitems = line.split() # Extract individual items.
        itemcount = len(lineitems)
        if itemcount > 2: # item 2 = number/name exists.
            pin = lineitems[2]
        else: # Incomplete command.
            self.RPi.Write("# pin command failed: No pin ID.")
            return False
        if itemcount > 3: # item 3 = command exists.
            command = lineitems[3].lower()
        else: # Incomplete command.
            self.RPi.Write("# pin command failed: No command.")
            return False
        if itemcount > 4: # item 4 = optional duration.
            duration = float(lineitems[4])
        else: duration = 0
        if itemcount > 5: # item 5 = repeat count.
            repeats = int(lineitems[5])
        else:
            repeats = 0
        pinobj = None
        for pins in GPIOpin.PinList: # Search the defined pins for the specified one.
            if pins.Name == str(pin): # Find by name.
                pinobj = pins # This is the pin instance we'll work with.
                break # Found it.
        if pinobj == None: # Failed to identify the pin.
            self.RPi.Write("# pin command failed: " + str(pin) + " unknown.")
            return False
        # We have a valid command.
        print(pinobj.Name,command,duration,repeats)
        for i in range(max(repeats,1)): # Loop as many times as requested.
            if command == 'on' and pinobj.Pin.direction == digitalio.Direction.OUTPUT: # Switch on and it's an OUTPUT. command
                pinobj.SetValue(True) # Turn on.
                if duration != 0: # For a limited time only.
                    time.sleep(duration) # Wait until time to turn off again.
                    pinobj.SetValue(False) # Turn off.
                    if repeats > 0: time.sleep(duration) # If we're going to repeat, pause before repeating too.
            elif command == 'off' and pinobj.Pin.direction == digitalio.Direction.OUTPUT: # Switch off and it's an INPUT.
                pinobj.SetValue(False) # Turn off.
                if duration != 0: # For a limited time only.
                    time.sleep(duration) # Wait until time to turn on again.
                    pinobj.SetValue(True) # Turn on.
                    if repeats > 0: time.sleep(duration) # If we're going to repeat, pause before repeating too.
            elif command == 'state': # Send pin state.
                self.RPi.Write("# pin status " + str(pinobj.Name) + " " + str(pinobj.Pin.value))
        return True

