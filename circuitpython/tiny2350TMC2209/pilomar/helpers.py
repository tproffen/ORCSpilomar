# Based on https://github.com/Short-bus/pilomar
# Simplified for use with TINY2350

#-----------------------------------------------------------------------------------------------
class GPIOpin():

    PinList = [] # Maintain a list of all defined pins.

    def __init__(self,pin,name=None):
        print("GPIOpin: Initializing pin",pin,name)
        self.Pin = digitalio.DigitalInOut(pin)
        self.PinNumber = pin
        self.Name = name
        self.PrevValue = self.GetValue()
        GPIOpin.PinList.append(self) # Add this pin to the list of defined pins.

    def SetDirection(self,direction):
        self.Pin.direction = direction
        if direction == digitalio.Direction.OUTPUT: self.Pin.value = False # turn off.

    def SetValue(self,value):
        if self.Pin.direction == digitalio.Direction.OUTPUT: self.Pin.value = value

    def GetValue(self):
        return self.Pin.value

    def High(self):
        return self.GetValue()

    def Low(self):
        return not self.GetValue()

    def Pressed(self):
        """ Return TRUE if the pin has just gone HIGH else FALSE. """
        result = False
        if self.Pin.value != self.PrevValue:
            if self.Pin.value: # Gone HIGH.
                result = True
            self.PrevValue = self.Pin.value
        return result

    def Released(self):
        """ Return TRUE if the pin has just gone LOW else FALSE. """
        result = False
        if self.Pin.value != self.PrevValue:
            if not self.Pin.value: # Gone LOW.
                result = True
            self.PrevValue = self.Pin.value
        return result

#-----------------------------------------------------------------------------------------------
class led():
    def __init__(self,pin,state=False):
        """ Tiny RGB LED on/off state is inverted!
            ie to turn LED ON, pin must go LOW.
               to turn LED OFF, pin must go HIGH. """
        self.Led = digitalio.DigitalInOut(pin)
        self.Led.direction = digitalio.Direction.OUTPUT
        self.Led.value = None # Off
        self.Enabled = True # Set to FALSE to turn off the LED completely.
        if state: self.On()
        else: self.Off()

    def Enable(self):
        """ Enable the LED and turn it on if required. """
        self.Enabled = True
        if self.Led.value: self.On()

    def Disable(self):
        """ Disable the LED and make sure it is turned off. """
        self.Enabled = False
        self.Off()

    def On(self):
        """ Turn the LED ON.
            NOTE: Tiny RGB LED uses opposite pin state to control LED. """
        if self.Enabled: self.Led.value = False
        else: self.Led.value = True

    def Off(self):
        """ Turn the LED OFF.
            NOTE: Tiny RGB LED uses opposite pin state to control LED. """
        self.Led.value = True

#-----------------------------------------------------------------------------------------------
class statusled():
    """ Pimoroni Tiny RGB LED version of RGB LED handling.
        The RGB LED is a collection of three led() objects. """
    def __init__(self):
        # Raspberry Pi Pico & Pico 2
        #self.LedR = led(board.GP18) # LED_R) # Create LED for RED channel.
        #self.LedG = led(board.GP19) # LED_G) # Create LED for GREEN channel.
        #self.LedB = led(board.GP20) # LED_B) # Create LED for BLUE channel.
        # Pimoroni Tiny...
        self.LedR = led(board.LED_R) # Create LED for RED channel.
        self.LedG = led(board.LED_G) # Create LED for GREEN channel.
        self.LedB = led(board.LED_B) # Create LED for BLUE channel.
        self.TaskList = {'idle': (False,False,False), # Off
                         'coms': (False,False,True), # Blue - Flashes when handling UART
                         'move': (False,True,False), # green - Flashes when motor is moving.
                         'error': (True,False,False), # Red - Indicates failure/fault.
                         'init': (False,True,True)} # cyan - System is initializing.
        self.Enabled = True # If set to FALSE, the LED is permanently off except for ERROR conditions.
        self.StatusExpiry = 0 # microsecond count when any error status can be cleared.
        self.Task('idle')

    def Enable(self):
        """ Enable the LED. """
        self.Enabled = True

    def Disable(self):
        """ Disable the LED. """
        self.Enabled = False
        self.Task('idle')

    def Task(self,task):
        """ Set LED color according to the task assigned.
            ERROR tasks always win, and an ERROR setting stays illuminated for at least 1 second
            before any other task can set any other colors. """
        ErrorTask = 'error'
        ms = int(time.monotonic_ns() / 1e9) # Seconds timer. Reduce by 1.0e9 to get seconds.
        if ms < self.StatusExpiry: # Error code is still valid. No other setting can be used yet.
            pass
        elif self.Enabled or task == ErrorTask: # No current ERROR status, so we can consider whether to set the new status.
            if task == ErrorTask: # Setting a new error status. Set the timeout for 1 second ahead.
                self.StatusExpiry = ms + 1 # Set 1 second expiry on error codes.
            if task in self.TaskList: # Pull the color codes for the selected task.
                t = self.TaskList[task]
                if t[0]: self.LedR.On()
                else: self.LedR.Off()
                if t[1]: self.LedG.On()
                else: self.LedG.Off()
                if t[2]: self.LedB.On()
                else: self.LedB.Off()
            else: # If the task is not recognised, the LEDs are turned off.
                self.LedR.Off()
                self.LedG.Off()
                self.LedB.Off()
        else: # LED is disabled for everything except ERRORS
            self.LedR.Off()
            self.LedG.Off()
            self.LedB.Off()

    def SetRGB(self,line):
        """ Receive a 'set rgb' command from the RPi and turn on
            the status LED in that color to acknowledge it.
            This is a debug/dev feature to let people prove that
            the Tiny is indeed receiving and processing commands.

            This works even if the LED is disabled!

            set rgb yyyymmddhhmmss y y y nnn
             0   1       2         3 4 5  6

            3 = y/n = RED ON
            4 = y/n = GREEN ON
            5 = y/n = BLUE ON
            6 = nnn = Minimum seconds to illuminate. """
        lineitems = line.split(' ')
        lli = len(lineitems)
        if lli > 3 and lineitems[3] == 'y': self.LedR.On()
        else: self.LedR.Off()
        if lli > 4 and lineitems[4] == 'y': self.LedG.On()
        else: self.LedG.Off()
        if lli > 5 and lineitems[5] == 'y': self.LedB.On()
        else: self.LedB.Off()
        if lli > 6:
            self.StatusExpiry = int(time.monotonic_ns() / 1e9) + int(lineitems[6]) # Set expiry on the LED color.# Write your code here :-)

#-----------------------------------------------------------------------------------------------
class exceptioncounter():
    """ Keep a count of how many exceptions have been raised during operation.
        All exceptions should be reported back to the RPi in the log messages
        however a simple counter also ensures we know when things are having
        problems. Implemented as a class so that it can be called consistently
        from inside other methods and objects.
        This will also automatically set the LED to the error color. """
    def __init__(self):
        self.Count = 0 # Initialize the counter.

    def Reset(self):
        """ Reset the exception count. """
        self.Count = 0 # Initialize the counter.

    def Raise(self):
        try:
            self.Count += 1 # Increment the count.
            StatusLed.Task('error') # Set the LED to show an error was trapped.
            print("Exception trapped.")
        except Exception as e:
            print("exceptioncounter.Raise() failed:",str(e))

#-----------------------------------------------------------------------------------------------
class timer():
    """ An event timer mechanism.
        Create with
            NAME = timer('heartbeat',repeat=30,offset=10)
                'heartbeat' = name of the timer.
                repeat = 30 seconds between events.
                offset = 1st event is current time + 10 seconds.
        Use with
            if NAME.Due(): print ('Event due')

        This returns TRUE if the timer has expired.
        It automatically resets the timer to repeat.

        MyTimer = timer('demo',30)
        while True:
            if MyTimer.Due(): print("another 30 seconds has passed.")

        """

    def __init__(self,name,repeat,offset=0):
        """ Create the timer.
            Initialize the first due time. """
        self.Name = name
        if repeat < 1: repeat = 1 # Minimum repeat cycle is 1 second.
        self.RepeatSeconds = repeat
        if offset != 0: self.NextDue = time.time() + offset # If the timer starts with an offset then that's the FIRST due time.
        else: self.NextDue = time.time() + repeat # There's no offset, so due time is calculated from now.

    def SetNextDue(self):
        """ If NextDue has expired, this calculates the next future value.
            It always rolls forward a whole number of 'repeat seconds' to the first due time that is in the future. """
        while self.NextDue <= time.time():
            gap = ((time.time() - self.NextDue) // self.RepeatSeconds) + 1 # How many multiples of 'repeatseconds' do we need to add to get to the next timeslot?
            if gap >= 0:
                self.NextDue = self.NextDue + (self.RepeatSeconds * gap)

    def Reset(self):
        """ Restart the time from NOW. """
        self.NextDue = time.time() + self.RepeatSeconds

    def Remaining(self):
        """ How many seconds left on the timer? """
        remaining = self.NextDue - time.time()
        return remaining

    def Due(self):
        """ This returns TRUE if the timer has expired.
            It returns FALSE if the timer is still running.
            If the timer expires, it also resets the timer for the next event. """
        result = False
        temp = self.Remaining()
        if temp > self.RepeatSeconds: # If the local clock changes, the timer may be left out of sync. Reset it.
            print("clock(",self.Name,") remaining",temp,"exceeds repeat",self.RepeatSeconds,". Timer reset.")
            self.NextDue = time.time() + self.RepeatSeconds
        elif temp <= 0: # Timer expired.
            result = True
            self.SetNextDue()
        return result

#-----------------------------------------------------------------------------------------------
class logfile():
    """ A simple logging mechanism.
        Record any log messages in a temporary list.
        When directed, send the list to the remote server
        for storage.

        MyLog = logfile()
        MyLog.Log("This is a message for the logfile")
        ...
        MyLog.SendAll()

        """
    def __init__(self):
        self.Lines = []
        self.BufferSize = 0
        self.MaxLines = 20 # Do not store more than 20 lines due to memory constraints.
        self.Overflows = 0 # How many times has the buffer filled?

    def Log(self,line,*args):
        """ Accept any number of arguments, convert them into a single log file message. """
        for x in args:
            if type(x) == type(str): a = x
            else: a = str(x) # Convert all additional items to strings and append them.
            line = (line + ' ' + a).strip()
        if self.Clock == None:
            line = IntToTimeString(time.time()) + ":" + line + '\n'
        else:
            line = IntToTimeString(self.Clock.Now()) + ":" + line + '\n'
        if len(self.Lines) < self.MaxLines:
            self.Lines.append(line)
            self.BufferSize += len(line)
        else:
            print("logfile.Log: Buffer is full. log message ignored until cleared.")
            self.Overflows += 1
            print("logfile.Log: Memory available",gc.mem_free(),"bytes.")

    def SendAll(self):
        """ Call this to send ALL the outstanding log entries. """
        for line in self.Lines:
            RPi.Write('log :' + line,log=False)
        self.Lines = []
        self.BufferSize = 0

    def SendOne(self):
        """ Call this to send a single log entry if the microcontroller is idle. """
        if len(self.Lines) > 0:
            line = self.Lines.pop(0)
            tottemp = 0
            for line in self.Lines: tottemp += len(line)
            self.BufferSize = tottemp
            RPi.Write('log :' + line,log=False)

    def SendCheck(self,force=False):
        """ If local buffer is large enough, send it to the host
            and reset ready for new messages. """
        if self.BufferSize > 80 or force:
            self.SendAll()

#-----------------------------------------------------------------------------------------------
class clock():
    """ Maintain a clock that is roughly in sync with the host server.
        If the microcontroller is running standalone, then its internal clock
        does not get synchronised. This class allows a 'timedelta' to be
        defined in the program which is applied to the internal clock to
        give an approximately current timestamp.
        If the microcontroller is linked to a development tool such as Thonny
        it may synchronise the internal clock, in which case the TimeDelta will
        not be used.
        If the Thonny connection is made AFTER the program has started, the
        TimeDelta value will no longer be needed when the internal clock itself
        is synchronised. If this is detected, TimeDelta is cleared and rechecked. """
    def __init__(self,TimeValue=None):
        self.TimeDelta = 0 # Offset from machine clock to current date/time (in seconds).
        self.ClockSynchronised = False # Indicates that the clock is synchronised.
        self.PrevTime = time.time() # Record the initial unmodified time of the clock. Used to detect if machine clock gets updated.
        if type(TimeValue) == type(str):
            self.SetTimeFromString(TimeValue)
        elif type(TimeValue) == type(int):
            self.SetTimeFromInt(TimeValue)
        # Values for the NowDecimal() function which simulates fractions of a second for timestamps.
        self.CurrTime = time.time() # What's the current integer timestamp?
        self.FirstNS = float(time.monotonic_ns()) # What's the nanosecond count at the start of the timestamp?
        self.MaxDecimal = 0.999999999 # Decimal portion cannot roll into next second.
        # Warn CP times stored as single int. Overflow after YEAR > 2033.
        if time.localtime(time.time())[0] > 2033:
            print("CLOCK WARNING: Potential int overflow after 2033.")

    def UpdateClockFromInt(self,TimeInt):

        """ Given any integer timestamp this will compare against the clock
            and update the clock if the new timestamp is AHEAD of the current
            clock. This increases the accuracy/synchronisation of the clock with the RPi's clock.
            This can be run against any received timestamp to continually improve the clock's time. """

        tn = self.Now() # What time does the microcontroller think it is at the moment?
        result = False
        if TimeInt != None and TimeInt > tn: # We can nudge the clock forward, never backwards.
            self.SetTimeFromInt(TimeInt)
            LogFile.Log("UpdateClockFromInt(",IntToTimeString(TimeInt),") replaces",IntToTimeString(tn),"Updated clock.")
            result = True
        return result

    def UpdateClockFromString(self,TimeString):

        """ Given any character timestamp this will compare against the clock
            and update the clock if the new timestamp is AHEAD of the current
            clock. This increases the accuracy/synchronisation of the clock with the RPi's clock.
            This can be run against any received timestamp to continually improve the clock's time.

            If the microcontroller is connected via USB to Thonny on a remote machine, the machine clock may then
            get synchronised, in which case the TimeDelta value is nolonger needed. """

        result = False
        for a in [' ','.','-',':']:
            TimeString = TimeString.replace(a,"")
        try:
            result = TimeStringToInt(TimeString)
            self.UpdateClockFromInt(result)
        except:
            LogFile.Log("UpdateClockFromString(",TimeString,") failed.")
            ExceptionCounter.Raise() # Increment exception count for the session.
        return result

    def CheckTimeDelta(self,now):
        """ If the basic clock time suddenly jumps, assume that the clock has been synchronised
            in which case clear TimeDelta because it's nolonger needed. """
        if now - self.PrevTime > 3600: # Clock has suddenly jumped an hour or more!
            print("c.CTD: Internal clock may have synchronised. Resetting clock synchronisation.")
            self.TimeDelta = 0
            self.ClockSynchronised = False
            LogFile.Log("CheckTimeDelta(): Internal clock may have synchronised. Resetting clock synchronisation.")
        self.PrevTime = now

    def SetTimeFromInt(self,TimeInt):
        """ Set clock offset from an INTEGER TIME. """
        self.TimeDelta = max(TimeInt - time.time(),0) # Time offset is the received time - the realtime clock time. Cannot be negative.
        self.ClockSynchronised = True
        RPi.Write('# Clock now ' + IntToTimeString(self.Now()) + ' timedelta ' + str(self.TimeDelta) + ' seconds.')
        #result = True

    def SetTimeFromString(self,TimeString):
        """ Set clock offset from a CHARACTER TIME. """
        result = False
        for a in [' ','.','-',':']:
            TimeString = TimeString.replace(a,"")
        try:
            result = self.SetTimeFromInt(TimeStringToInt(TimeString))
        except Exception as e:
            LogFile.Log('clock.SetTimeFromString: Invalid timestamp string (', TimeString, ')')
            ExceptionCounter.Raise() # Increment exception count for the session.
        return result

    def Now(self):
        """ Return current clock time. As micropython number of seconds."""
        now = time.time()
        self.CheckTimeDelta(now) # If the internal clock has recently been synchronised clear TimeDelta until it can be recalculated.
        return (now + self.TimeDelta)

    def NowDecimal(self):
        """ Returns 2 values.
            1) The traditional integer time.time() value.
            2) A matching pseudo fraction of a second,
               this is calculated using time.monotonic_ns() to measure
               nanoseconds and estimate how many nanoseconds have elapsed
               since the current 'second' first occurred. """
        CurrDecimal = 2.0
        while CurrDecimal >= 1.0: # Decimal must be within the current second.
            while self.Now() != self.CurrTime: # If the clock has rolled on to a new second, calculate a ZERO point for the monotonic_ns() clock.
                self.FirstNS = float(time.monotonic_ns()) # This is the ZERO point for nanoseconds within this second.
                self.CurrTime = self.Now() # The current clock time.
            CurrDecimal = (float(time.monotonic_ns()) - self.FirstNS) / 1e+9 # Elapsed fraction of a second since is started.
        return self.CurrTime, CurrDecimal

    def NowString(self):
        """ Return current clock time. As character string. """
        return IntToTimeString(self.Now())

#-----------------------------------------------------------------------------------------------
# Various conversion routines
#-----------------------------------------------------------------------------------------------

def BoolToString(value):
    if value: result = 'y'
    else: result = 'n'
    return result

def StringToBool(value,default=False):
    if value.lower() == 'y' or value.lower() == 'true': result = True
    elif value.lower() == 'n' or value.lower() == 'false': result = False
    else: result = default
    return result

def IntToTimeString(timestamp):
    # Sometime around 2034, this may cause problems for localtime() method if it generates longint values.
    # int values max out at (2^30) - 1 = 1073741823
    lt = time.localtime(timestamp)
    entry = ('0000' + str(lt[0]))[-4:] # Year
    entry += ('00' + str(lt[1]))[-2:] # Month
    entry += ('00' + str(lt[2]))[-2:] # Day
    entry += ('00' + str(lt[3]))[-2:] # Hour
    entry += ('00' + str(lt[4]))[-2:] # Minute
    entry += ('00' + str(lt[5]))[-2:] # Second
    return entry# Write your code here :-)

def TimeStringToInt(timestamp):
    # Sometime around 2034, this may cause problems for mktime() method if it generates longint values.
    # int values max out at (2^30) - 1 = 1073741823
    year = int(timestamp[0:4])
    month = int(timestamp[4:6])
    day = int(timestamp[6:8])
    hour = int(timestamp[8:10])
    minute = int(timestamp[10:12])
    second = int(timestamp[12:14])
    result = time.mktime((year,month,day,hour,minute,second,0,-1,-1))
    return result

def IsFloat(text):
    """ Return TRUE if a string can be converted to a float value. """
    result = False
    try:
        _ = float(text)
        result = True
    except ValueError:
        pass
        # Don't increment exception count for this one.
    return result

def SendCpuStatus():
        """ Report microcontroller condition back to RPi. """
        line = 'cpu status ' + IntToTimeString(Clock.Now()) + ' '
        line += str(microcontroller.cpus[0].reset_reason).split('.')[-1].replace(' ','_') + ' '
        line += str(microcontroller.cpus[0].frequency / 1e6) + ' '
        line += '0.0 ' # s/b 'volt: ' + str(microcontroller.cpus[0].voltage) + ', '
        line += str(gc.mem_alloc()) + ' ' + str(gc.mem_free()) + ' '
        line += str(int(microcontroller.cpus[0].temperature)) + ' '
        RPi.Write(line)

def ns_sleep(delay=1.0):
        """ Provide a SLEEP function which supports pauses less than 1 millisecond.
            Standard time.sleep() does not seem to work below 1 millisecond.
            at 200MHz this can achieve delays down to 0.00022 seconds. 5 times shorter than time.sleep() """
        if delay < 0.001: # Very short delays don't work with the standard function.
            t1 = time.monotonic_ns() + (delay * 1e9) # When does the delay expire?
            while time.monotonic_ns() < t1:
                pass # Loop until clock reaches the set time.
        else: # Millisecond and above delays can use standard function.
            time.sleep(delay)
        return True

def check_version():
    Bootline = '' # Make sure the entire boot_out.txt content is available as a single item.
    CircuitPythonVersion = ''
    with open('boot_out.txt','r') as f: # Read the configuration summary.
        while True: # Loop through all the lines in turn.
            line = f.readline()
            if line == '': break # End of file.
            lines = line.split(';') # Split the line.
            for item in lines:
                cleanitem = item.strip() # Remove unwanted characters.
                Bootline += cleanitem + ' '
                for elements in item.split(' '): # Split into individual elements.
                    if elements.strip().lower() == 'circuitpython': CircuitPython = True # This is a CircuitPython build.
                    if len(elements.split('.')) > 2: # 'a.b.c' and 'a.b.c-alpha.2350' formas are version number.
                        CircuitPythonVersion = elements # Allows code to adapt to different CircuitPython versions.

    return (CircuitPythonVersion, Bootline)

#-----------------------------------------------------------------------------------------------
# Feature dependent routines
#-----------------------------------------------------------------------------------------------
if 'VMOT' in FEATURES:
    # Use ADC to read VMOT value. We can detect if motors are actually powered.
    # - Losing power is an easy problem to detect and report.
    # - Also can give clues if running off batteries and they are running low.
    import analogio
    VMotADC = analogio.AnalogIn(board.A0)

    def VMot():
        """ Read the current MotorPower ADC value directly.
            Don't scale for voltage, let the host deal with that. """
        result = VMotADC.value
        return result

if 'TINY' in FEATURES:
    BootBCM = GPIOpin(board.USER_SW,'boot') # Tiny RP2040 # The BOOTSEL button.
    BootBCM.SetDirection(digitalio.Direction.INPUT)
    BootBCM.pull = digitalio.Pull.UP # The button goes LOW when pressed.

if 'RP2350' in FEATURES:
    microcontroller.cpu.frequency = 200000000 # Set to 200MHz (default 150MHz on Tiny2350)
    print("Set clock frequency on",board.board_id,
      "from:",(temp / 1e6),"MHz",
      "to:",(microcontroller.cpu.frequency / 1e6),"MHz")

