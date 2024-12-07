# Based on https://github.com/Short-bus/pilomar
# Simplified for use with TINY2350

import microcontroller
import time
import gc

from pilomar.helpers import *

#-----------------------------------------------------------------------------------------------
class picosession():
    def __init__(self, LogFile, ExceptionCounter, Clock, Motors, RPi, CircuitPythonVersion):
        self.Clock = Clock
        self.Motors = Motors
        self.LogFile = LogFile
        self.ExceptionCounter = ExceptionCounter
        self.RPi = RPi
        self.CircuitPythonVersion = CircuitPythonVersion
        
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
        for i in self.Motors: # for ALL motors.
            if not i.MotorConfigured: result = False # Motor must be configured.
        self.RemoteControl = result
        # Decide if the microcontroller can have autonomous control of the motors.
        # They may start moving immediately.
        if not self.Clock.ClockSynchronised: result = False # Clock must be synchronised.
        for i in self.Motors: # for ALL motors.
            if not i.Trajectory.Valid: result = False # Trajectory must be valid.
        self.AutonomousControl = result

    def SendMotorStatus(self,motorname,immediate=False,codes='?-?'):
        """ Decide which motor status to send.
            immediate: True: Status is sent even if not due.
                       False: Status is only sent if timer is due.
            codes: Optional extra codes added to the status message (dev/test/debug etc) """
        for i in self.Motors:
            if i.MotorName == motorname: i.SendMotorStatus(immediate=immediate,codes=codes)

    def TrajectorySafety(self):
        """ If the remote RPi crashes while a trajectory is underway but leaves the microcontroller powered
            the microcontroller will continue to follow the trajectory until it expires, this could be 20+ minutes.
            For safety, clear trajectories of all motors if communication has stalled.
            If communication resumes, the RPi will send the trajectory again. """
        failsafe = False
        try:
            elapsed = self.RPi.ticks_ms() - self.RPi.LastRxms # How many ms elapsed since last receipt?
        except:
            elapsed = 0
            self.LogFile.Log('TrajectorySafety: elapsed calculation failed.',self.RPi.ticks_ms(),self.RPi.LastRxms)
            self.ExceptionCounter.Raise() # Increment exception count for the session.
        if self.TrajectorySafetyms != None and elapsed > self.TrajectorySafetyms: # No messages received recently.
            for i in self.Motors:
                if i.Trajectory.Valid: # There's a trajectory underway.
                    failsafe = True # Trigger failsafe activity.
        if failsafe and self.FailsafeLatch == False:
            self.LogFile.Log('TrajectorySafety:',elapsed,'ms, failsafe?')
            self.FailsafeLatch = True # Don't let this message repeat continually.
            self.TrajectorySafetyFlushes += 1 # Increment the number of times we've flushed the trajectories for safety.
            for i in self.Motors:
                i.ClearTrajectory() # Flush the trajectory from each motor for safety.
        if failsafe == False: self.FailsafeLatch = False # Reset the latch.

    def SendSessionStatus(self,codes='?-?'):
        """ Generate status message to RPi.
            The RPi uses this to decide what commands and configurations to send to the microcontroller.
            This can send multipleitems to the RPi, they are sent individually in sequence rather than as
            a single large packet of everything. Smaller messages work more reliably.
            codes: Optional extra flags added to status message (dev/test/debug etc)
            """
        if self.CircuitPythonVersion.split('.')[0] in ['7','8','9']: pass # Supported CircuitPython version.
        else: # Unexpected CircuitPython version, report it back.
            line = '# Expecting CircuitPython 7,8 or 9, found ' + str(self.CircuitPythonVersion)
            self.RPi.Write(line) # Send over UART to RPi.
        line = "session status "
        i = time.time() - self.RPi.StartTime # Alive seconds. Use CPU clock not synchronised clock.
        line += IntToTimeString(self.Clock.Now()) + ' ' # Current local timestamp.
        line += BoolToString(self.Clock.ClockSynchronised) + ' ' # Do the RPi and microcontroller clocks agree?
        line += BoolToString(self.AutonomousControl) + ' '  # Can motors drive themselves? Fully configured and trajectory known.
        line += BoolToString(self.RemoteControl) + ' '  # Can motors be commanded remotely? Fully configured.
        line += str(i) + ' ' # Alive seconds. Use CPU clock, not synchronised clock.
        line += str(self.TrajectorySafetyFlushes) + ' ' # How many times has the trajectory been flushed for safety when comms failed?
        line += str(codes) + ' ' # Add optional extra codes.
        line += str(self.ExceptionCounter.Count) + ' ' # Append count of exceptions raised during operation.
        self.RPi.Write(line) # Send over UART to RPi.
        line = "comms status "
        line += IntToTimeString(self.Clock.Now()) + ' ' # Current local timestamp.
        line += str(self.RPi.PicoRxErrors) + ' '  # How many messages were rejected from RPi by Microcontroller.
        line += str(self.RPi.CharactersRead) + ' '  # How many bytes received from RPi by Microcontroller.
        line += str(self.RPi.CharactersWritten) + ' '  # How many bytes written by Microcontroller to RPi.
        line += str(self.RPi.WriteDrops) + ' '  # How many messages were dropped due to buffer overflow?
        line += str(self.RPi.ReceiveAge()) + ' ' # Report how old the last received message is...
        line += str(self.RPi.ReadDrops) + ' ' # How many received messages have been dropped because input buffer was full?
        line += str(len(self.RPi.WriteQueue)) + ' ' # How many messages in the send queue currently? Checking for backlog building up.
        line += str(codes) + ' ' # Add optional extra codes.
        self.RPi.Write(line) # Send over UART to RPi.

    def AutoMoveMotors(self): # Trigger movement of the motors.
        """ Call this to check the current position of each motor against their trajectory.
            If the motor needs to move, this will perform the motion. """
        overallresult = False
        self.MovePermission() # Is motor still capable of autonomous movement?
        if self.AutonomousControl:
            overallresult = True
            for i in self.Motors:
                result = i.TargetFromTrajectoryPosition() # Set target for the motor based upon trajectory if available.
                if result: # Target was successfully set.
                    if i.TargetPosition != i.CurrentPosition: i.MoveMotor()
                else: # Target was not successfuly set.
                    self.LogFile.Log('AutoMoveMotors',i.MotorName,'failed: TargetFromTrajectory returned', result)
                    overallresult = False
        return overallresult

    def SendCpuStatus(self):
        """ Report microcontroller condition back to RPi. """
        line = 'cpu status ' + IntToTimeString(self.Clock.Now()) + ' '
        line += str(microcontroller.cpus[0].reset_reason).split('.')[-1].replace(' ','_') + ' '
        line += str(microcontroller.cpus[0].frequency / 1e6) + ' '
        line += '0.0 ' # s/b 'volt: ' + str(microcontroller.cpus[0].voltage) + ', '
        line += str(gc.mem_alloc()) + ' ' + str(gc.mem_free()) + ' '
        line += str(int(microcontroller.cpus[0].temperature)) + ' '
        self.RPi.Write(line)

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

