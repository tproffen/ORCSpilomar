# Based on https://github.com/Short-bus/pilomar
# Simplified for use with TINY2350

#-----------------------------------------------------------------------------------------------
class picosession():
    def __init__(self):
        self.SessionStart = time.time()
        self.AutonomousControl = False # Triggers movement of the motors when they are configured and trajectories loaded.
        self.RemoteControl = False # Allows movement of the motors when they are configured, regardless of trajectories existing.
        self.Quit = False # Set to TRUE to terminate the session.
        self.TrajectorySafetyms = 2 * 60 * 1000 # How many milliseconds can a valid trajectory remain in use before comms failure terminates it? == 2 minutes.
        self.TrajectorySafetyFlushes = 0 # How many times have we had to flush the trajectories for safety when comms seemed to fail?
        self.FailsafeLatch = False # Latch to prevent 'failsafe' messages flooding the communication buffers when safety flush is triggered.

    def MovePermission(self):
        """ Decide if the microcontroller can accept remote control of the motors.
            They will move under the direction of the remote RPi. """
        result = True
        for i in Motors: # for ALL motors.
            if not i.MotorConfigured: result = False # Motor must be configured.
        self.RemoteControl = result
        # Decide if the microcontroller can have autonomous control of the motors.
        # They may start moving immediately.
        if not Clock.ClockSynchronised: result = False # Clock must be synchronised.
        for i in Motors: # for ALL motors.
            if not i.Trajectory.Valid: result = False # Trajectory must be valid.
        self.AutonomousControl = result

    def SendMotorStatus(self,motorname,immediate=False,codes='?-?'):
        """ Decide which motor status to send.
            immediate: True: Status is sent even if not due.
                       False: Status is only sent if timer is due.
            codes: Optional extra codes added to the status message (dev/test/debug etc) """
        for i in Motors:
            if i.MotorName == motorname: i.SendMotorStatus(immediate=immediate,codes=codes)

    def TrajectorySafety(self):
        """ If the remote RPi crashes while a trajectory is underway but leaves the microcontroller powered
            the microcontroller will continue to follow the trajectory until it expires, this could be 20+ minutes.
            For safety, clear trajectories of all motors if communication has stalled.
            If communication resumes, the RPi will send the trajectory again. """
        failsafe = False
        try:
            elapsed = RPi.ticks_ms() - RPi.LastRxms # How many ms elapsed since last receipt?
        except:
            elapsed = 0
            LogFile.Log('TrajectorySafety: elapsed calculation failed.',RPi.ticks_ms(),RPi.LastRxms)
            ExceptionCounter.Raise() # Increment exception count for the session.
        if self.TrajectorySafetyms != None and elapsed > self.TrajectorySafetyms: # No messages received recently.
            for i in Motors:
                if i.Trajectory.Valid: # There's a trajectory underway.
                    failsafe = True # Trigger failsafe activity.
        if failsafe and self.FailsafeLatch == False:
            LogFile.Log('TrajectorySafety:',elapsed,'ms, failsafe?')
            self.FailsafeLatch = True # Don't let this message repeat continually.
            self.TrajectorySafetyFlushes += 1 # Increment the number of times we've flushed the trajectories for safety.
            for i in Motors:
                i.ClearTrajectory() # Flush the trajectory from each motor for safety.
        if failsafe == False: self.FailsafeLatch = False # Reset the latch.

    def SendSessionStatus(self,codes='?-?'):
        """ Generate status message to RPi.
            The RPi uses this to decide what commands and configurations to send to the microcontroller.
            This can send multipleitems to the RPi, they are sent individually in sequence rather than as
            a single large packet of everything. Smaller messages work more reliably.
            codes: Optional extra flags added to status message (dev/test/debug etc)
            """
        if CircuitPythonVersion.split('.')[0] in ['7','8','9']: pass # Supported CircuitPython version.
        else: # Unexpected CircuitPython version, report it back.
            line = '# Expecting CircuitPython 7,8 or 9, found ' + str(CircuitPythonVersion)
            RPi.Write(line) # Send over UART to RPi.
        line = "session status "
        i = time.time() - RPi.StartTime # Alive seconds. Use CPU clock not synchronised clock.
        line += IntToTimeString(Clock.Now()) + ' ' # Current local timestamp.
        line += BoolToString(Clock.ClockSynchronised) + ' ' # Do the RPi and microcontroller clocks agree?
        line += BoolToString(self.AutonomousControl) + ' '  # Can motors drive themselves? Fully configured and trajectory known.
        line += BoolToString(self.RemoteControl) + ' '  # Can motors be commanded remotely? Fully configured.
        line += str(i) + ' ' # Alive seconds. Use CPU clock, not synchronised clock.
        line += str(self.TrajectorySafetyFlushes) + ' ' # How many times has the trajectory been flushed for safety when comms failed?
        line += str(codes) + ' ' # Add optional extra codes.
        line += str(ExceptionCounter.Count) + ' ' # Append count of exceptions raised during operation.
        RPi.Write(line) # Send over UART to RPi.
        line = "comms status "
        line += IntToTimeString(Clock.Now()) + ' ' # Current local timestamp.
        line += str(RPi.PicoRxErrors) + ' '  # How many messages were rejected from RPi by Microcontroller.
        line += str(RPi.CharactersRead) + ' '  # How many bytes received from RPi by Microcontroller.
        line += str(RPi.CharactersWritten) + ' '  # How many bytes written by Microcontroller to RPi.
        line += str(RPi.WriteDrops) + ' '  # How many messages were dropped due to buffer overflow?
        line += str(RPi.ReceiveAge()) + ' ' # Report how old the last received message is...
        line += str(RPi.ReadDrops) + ' ' # How many received messages have been dropped because input buffer was full?
        line += str(len(RPi.WriteQueue)) + ' ' # How many messages in the send queue currently? Checking for backlog building up.
        line += str(codes) + ' ' # Add optional extra codes.
        RPi.Write(line) # Send over UART to RPi.

    def AutoMoveMotors(self): # Trigger movement of the motors.
        """ Call this to check the current position of each motor against their trajectory.
            If the motor needs to move, this will perform the motion. """
        overallresult = False
        self.MovePermission() # Is motor still capable of autonomous movement?
        if self.AutonomousControl:
            overallresult = True
            for i in Motors:
                result = i.TargetFromTrajectoryPosition() # Set target for the motor based upon trajectory if available.
                if result: # Target was successfully set.
                    if i.TargetPosition != i.CurrentPosition: i.MoveMotor()
                else: # Target was not successfuly set.
                    LogFile.Log('AutoMoveMotors',i.MotorName,'failed: TargetFromTrajectory returned', result)
                    overallresult = False
        return overallresult

#-----------------------------------------------------------------------------------------------
class memorymanager():
    def __init__(self):
        self.currmem = None # Current memory free value.
        self.GCCount = 0 # How often has garbage collector run?
        self.Poll()

    def Poll(self): # Check current memory and trigger memory garbage collection early if needed.
        """ It looks like CircuitPython allocates memory in 2K chunks, it will error out if it cannot allocate 2K at a time.
            So run cleanup at 3K for safety. """
        self.currmem = gc.mem_free()
        if self.currmem < 3000:
            gc.collect()
            self.GCCount += 1 # Increase count of garbagecollector runs.

