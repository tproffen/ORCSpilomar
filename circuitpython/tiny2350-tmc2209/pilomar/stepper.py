# Based on https://github.com/Short-bus/pilomar
# Simplified for use with TINY2350

import analogio

from pilomar.tmc2209 import *
from pilomar.enum import *
from pilomar.helpers import *
from pilomar.trajectory import *

AzimuthStepBCM = GPIOpin(board.GP29,'azstep')
AltitudeStepBCM = GPIOpin(board.GP28,'altstep')
CommonDirectionBCM = GPIOpin(board.GP3,'dir')
CommonEnableBCM = GPIOpin(board.GP2,'enable')
AzimuthFaultBCM = GPIOpin(board.GP6,'azfault')
AltitudeFaultBCM = GPIOpin(board.GP7,'altfault')

VMotADC = analogio.AnalogIn(board.GP26)

TMCuart = busio.UART(board.GP4, board.GP5, baudrate=115200)

#-----------------------------------------------------------------------------------------------
# Configuration
#-----------------------------------------------------------------------------------------------

AzimuthStepBCM.SetDirection(digitalio.Direction.OUTPUT)
AzimuthStepBCM.SetValue(False)
AltitudeStepBCM.SetDirection(digitalio.Direction.OUTPUT)
AltitudeStepBCM.SetValue(False)
CommonDirectionBCM.SetDirection(digitalio.Direction.OUTPUT)
CommonDirectionBCM.SetValue(False)
CommonEnableBCM.SetDirection(digitalio.Direction.OUTPUT)
CommonEnableBCM.SetValue(False)
AzimuthFaultBCM.SetDirection(digitalio.Direction.INPUT)
AzimuthFaultBCM.pull = digitalio.Pull.UP
AltitudeFaultBCM.SetDirection(digitalio.Direction.INPUT)
AltitudeFaultBCM.pull = digitalio.Pull.UP

#-----------------------------------------------------------------------------------------------
class steppermotor():
    """ Handler for a NEMA17 stepper motor with TMC2209 driver chip. """

    def __init__(self, name, LogFile, ExceptionCounter, StatusLed, Clock, RPi, id=0):
        self.mtr_id = id
        self.LogFile = LogFile
        self.ExceptionCounter = ExceptionCounter
        self.StatusLed = StatusLed
        self.Clock = Clock
        self.RPi = RPi
        self.Setup_TMC()

        self.MotorName = name
        self.DriverType = 'tmc2209'
        self.Trajectory = trajectory(name, self.LogFile, self.ExceptionCounter, self.Clock)
        self.MotorEnabled = False
        self.CurrentPosition = None
        self.TargetPosition = None
        self.MotorConfigured = False
        self.FaultSensitive = False # Set to TRUE to monitor the 'fault' pin on the DRV8825 chip.
        self.FaultDetected = False # Latch to indicate we've already reported a fault with the DRV8825. Otherwise we overflow the UART comms buffer with warnings.
        self.SendStatus = True # Set to FALSE to disable status messages while downloading batches of data (eg Trajectories)
        self.StatusTimer = timer(name,10) # Set up an internal timer for sending status messages every 10 seconds. Can we overridden by RPi.
        self.FastTime = 0.0005 # The fastest pulse time for moving the motor.
        self.SlowTime = 0.05 # The slowest pulse time for moving the motor.
        self.DeltaTime = 0.003 # The acceleration amount moving from SlowTime to FastTime as the motor gets going.
        self.WaitTime = self.SlowTime # The time between pulses, starts slow, reduces as a move progresses. Resets every time a new target is set.
        self.Orientation = 1 # 1=Fwd, -1=Bkwd. This sets the overall orientation of motion. Compensate for gearing reversing the direction of motion here! It's applied to the DirectionBCM pin when the move is made.
        self.StepDir = 1 # The direction of a particular move +/-1. It always represents a SINGLE FULL STEP.
        self.LastStepDir = 0 # Record the 'last' direction that the motor moved in. Useful for handling gear backlash and handling rotation limits. Starts at ZERO (No direction)
        self.BacklashAngle = 0.0 # This is the angle the motor must move to overcome backlash in the gearing when changing direction.
        self.DriftSteps = 0 # This is the number of steps 'error' that DriftTracking has identified. It must be incorporated back into motor movements as smoothly as possible. Consider backlash etc.
        self.MotorStepsPerRev = None
        self.SlewStepMultiplier = 1 # The number of steps/microsteps taken when making large SLEW moves.
        self.SlewMotor = False # Allow the motor to make faster full-step (but less precise) moves when slewing the telescope large angles.
        self.GearRatio = None # gearratio is the overall gearing of the entire transmission. 60 means 60 motor revs for 1 transmission rev.
        self.AxisStepsPerRev = None
        self.MinAngle = None # Max anticlockwise movement.
        self.MaxAngle = None # Max clockwise movement.
        self.MinPosition = None # Min clockwise movement. Location of limit switch in steps (This is self calibrating when in use).
        self.MaxPosition = None # Max clockwise movement. Location of limit switch in steps (This is self calibrating when in use).
        self.RestAngle = None # The 'rest' position of the axis. Used for calibrating at startup. Typically DUE SOUTH or HORIZONTAL position of the axis.
        self.RestPosition = None
        # Limit position. This is not the position of a Limit switch, it is a rotation limit to prevent excessive cable twisting.
        self.LimitPosition = None # If given, this is the limit of movement. The telescope will reverse around this rather than cross it.
        self.RequestedPosition = None
        # Set up control pins for the motor.
        # If we're using the same pins to drive multiple motors you may get some warnings from GPIO.
        # If the program has been restarted in the same session, you may get some warnings from GPIO.
        self.StepBCM = None # Set pin to OUTPUT. This sends the MOVE pulse to the controller.
        self.DirectionBCM = None # Set pin to OUTPUT. This sets the MOVE direction to the controller.
        self.EnableBCM = None # Set pin to OUTPUT. This enables/disabled the motor.
        self.FaultBCM = None # Set pin to INPUT. Will EARTH when triggered.
        # The following items just need allocating, the self.Reset() call will set the values.
        self.OnTarget = False # Indicates that the motor is on target. This will control Observable status.
        self.LatestTuneSteps = 0 # Record details of the last tune command received. So we can see it was handled.
        self.LatestTuneTime = None
        # Latest Start/Stop times for config and status methods.
        self.OptimiseMoves = False # When set to TRUE the motor is allowed to take a short-cut if a requested move is > 50% of the circumference.


    def Setup_TMC(self):
        """ Setting default driver values via UART. """
        self.tmc=TMCStepper(TMCuart, self.LogFile, self.ExceptionCounter, mtr_id=self.mtr_id)
        
        self.tmc.set_interpolation(True)
        self.tmc.set_spreadcycle(False)
        self.tmc.set_internal_rsense(False)
                
    def CheckOnTarget(self):
        """ Set the OnTarget indicator if it looks like we're on-target.
            This means the REQUESTED POSITION = ACTUAL POSITION.
            (Don't use ANGLE because there may be tiny differences) """
        if self.RequestedPosition == None: self.OnTarget = False
        elif self.RequestedPosition == self.CurrentPosition: self.OnTarget = True
        else: self.OnTarget = False

    def Reset(self,enable=False):
        """ This resets the status of the motor.
            It does not physically move it, but it disables it, and sets the 'current position' to be the 'home position'.
            This is typically used for manually positioning a motor during initial setup, or for
            clearing the trajectory when selecting a new observation target.
            A fresh configuration will then be required from the RPi.
        """
        print("steppermotor.Reset(): Start")
        self.LogFile.Log(self.MotorName + ".Reset.")
        self.Trajectory.Clear() # Delete the trajectory completely. We'll be needing a new one.
        if enable: self.EnableMotor() # Leave motor enabled.
        else: self.DisableMotor() # Disable the motor.
        self.OnTarget = False
        self.CurrentPosition = self.RestPosition # Initialise CurrentPosition, we'll set it in next line. This is updated DURING moves too.
        self.TargetPosition = self.RestPosition
        self.RequestedPosition = None
        self.MotorConfigured = False
        self.SendStatus = True
        self.MotorStepsPerRev = None
        self.SlewStepMultiplier = 1 # The number of steps/microsteps taken when making large SLEW moves.
        self.SlewMotor = False # Allow the motor to make faster full-step (but less precise) moves when slewing the telescope large angles.
        self.GearRatio = None # gearratio is the overall gearing of the entire transmission. 60 means 60 motor revs for 1 transmission rev.
        self.AxisStepsPerRev = None
        print("steppermotor.Reset(): Send status...")
        self.SendMotorStatus(immediate=True,codes='rst')
        print("steppermotor.Reset(): End")
        return True

    def VMot(self):
        """ Return Motor voltage """
        result = VMotADC.value
        return result
    
    def CurrentDegrees(self):
        """ Return CurrentPosition as an angle. """
        if self.AxisStepsPerRev == None:
            cd = None # Cannot calculate until axis configuration is known.
        else:
            cd = 360.0 * (float(self.CurrentPosition) / self.AxisStepsPerRev)
        return cd

    def AddTrajectoryPoint(self,entry):
        try:
            self.Trajectory.Add(entry)
        except Exception as e:
            self.LogFile.Log('steppermotor(' + self.MotorName + ').AddTrajectoryPoint: ' + str(entry) + ': Failed. ' + str(e))
            self.ExceptionCounter.Raise() # Increment exception count for the session.
        self.SendMotorStatus(immediate=True,codes='atp') # This triggers the next trajectory point faster than waiting for the regular status message will.

    def ClearTrajectory(self):
        """ Remove current trajectory. """
        self.Trajectory.Clear() # Empty the entire trajectory.
        self.SendMotorStatus(immediate=True,codes='clt')

    def Stop(self):
        """ Immediately stop the motor. """
        self.ClearTrajectory()
        self.TargetPosition = self.CurrentPosition
        self.OnTarget = False
        self.RequestedPosition = None
        self.LogFile.Log('steppermotor.Stop(' + self.MotorName + ')')

    def EnableMotor(self):
        """ This engages current to the motor. It will hold its position now. """
        if self.MotorConfigured:
            self.EnableBCM.SetValue(False) # (1) # Pull pin LOW to enable.
            self.MotorEnabled = True
        else:
            self.LogFile.Log('steppermotor.EnableMotor: Motor is not configured. Will not enable.')

    def DisableMotor(self):
        """ This disengages current to the motor. It will not hold its position now. """
        self.EnableBCM.SetValue(True) # (0) # Pull pin HIGH to disable.
        self.MotorEnabled = False

    def ChangeSteps(self):
        """ Return proposed number of steps to move. """
        return self.TargetPosition - self.CurrentPosition

    def ExpectedPosition(self):
        """ What position should the motor be at according to the current trajectory. """
        position = self.Trajectory.ExpectedPosition()
        return position

    def GoToAngle(self,newangle):
        if self.MotorConfigured:
            self.LogFile.Log('steppermotor.GoToAngle(' + self.MotorName + ') ' + str(newangle))
            self.Stop() # Clear any pre-existing trajectory before moving.
            self.SetTargetByPosition(self.AngleToStep(newangle))
            self.MoveMotor()
        else:
            self.LogFile.Log('steppermotor.GoToAngle(' + self.MotorName + ') Rejected. Motor is not configured.')
            self.RPi.Write('goto rejected ' + self.MotorName + ' ' + str(newangle) + ' MotorNotConfigured')

        self.SendMotorStatus(immediate=True,codes='gte') # Tell RPi latest condition of the motor.

    def SetTargetByPosition(self,newposition=0,Limit=True):
        """ Set a new target POSITION (and therefore angle) for the motor.
            This will not move the motor, it will just prepare the step count and direction etc.
            newposition = The target position for the motor.
            Limit = True: The position must remain within limits.
            Limit = False: The position is not restricted at all. (Used for tuning.) """
        # Limit new angle to movement range. (MaxAngle, MinAngle)
        newangle = self.StepToAngle(newposition)
        result = True # Set succeeded.
        self.RequestedPosition = newposition # What position was requested?
        self.EnableMotor() # Enable the motor.
        if Limit: # OK to apply movement limits.
            if newangle > self.MaxAngle:
                self.LogFile.Log(self.MotorName + ": SetNewTarget: " + str(newangle) + " exceeds MaxAngle. Limited to: " + str(self.MaxAngle))
                newangle = self.MaxAngle
                newposition = self.AngleToStep(newangle)
                result = False # Set failed.
            if newangle < self.MinAngle:
                self.LogFile.Log(self.MotorName + ": SetNewTarget: " + str(newangle) + " exceeds MinAngle. Limited to: " + str(self.MinAngle))
                newangle = self.MinAngle
                newposition = self.AngleToStep(newangle)
                result = False # Set failed.
        self.TargetPosition = newposition # Convert it into the nearest absolute STEP position.
        self.WaitTime = self.SlowTime # Start with slow move pulses. This reduces each time we call StepMove().
        if self.ChangeSteps() > 0: self.StepDir = 1 # Which direction do we move in?
        else: self.StepDir = -1
        return result

    def TargetFromTrajectoryPosition(self):
        """ Establish the latest target angle from the current trajectory.
            Calculates the target position and sets up for the move. """
        result = False # Failed unless successful.
        targetposition = self.ExpectedPosition()
        print("TargetFromTrajectoryPosition=",targetposition)
        if targetposition != None and self.Trajectory.Valid and self.MotorConfigured: # We can set the target based upon current trajectory.
            result = self.SetTargetByPosition(targetposition)
        else: # Target is just the current position. Config and Trajectory are invalid.
            self.LogFile.Log('TargetFromTrajectoryPosition',self.MotorName,'not ready. tv,mc,ta=', self.Trajectory.Valid, self.MotorConfigured,targetposition)
        return result

    def TunePosition(self,delta):
        """ Tune the motor position. This shifts the motor, but retains the 'position' calculation unchanged.
            Use this to address positioning errors or drift adjustments. """
        if self.MotorConfigured:
            self.EnableMotor()
            tunestarttime = self.Clock.Now()
            old = self.CurrentPosition # Store the current position of the motor. We'll restore this when finished.
            new = self.CurrentPosition + delta # Calculate the new target position (fullsteps).
            self.SetTargetByPosition(new,Limit=False) # Set the target in the object. Primes it for the move, No error check on limits.
            self.LogFile.Log("TunePosition(" + self.MotorName + ") Current:" + str(self.CurrentPosition) + ", NewTarget: " + str(self.TargetPosition))
            self.MoveMotor() # Perform the move.
            self.CurrentPosition = old
            self.TargetPosition = old
            self.LogFile.Log("TunePosition(" + self.MotorName + ") set to " + str(self.CurrentPosition))
            self.LatestTuneSteps = delta # Record details of the last tune command received. So we can see it was handled.
            self.LatestTuneTime = self.Clock.Now()
            self.RPi.Write('tune complete ' + self.MotorName + ' ' + IntToTimeString(self.LatestTuneTime) + ' ' + str(delta) + ' ' + IntToTimeString(tunestarttime))
            self.self.SendMotorStatus(immediate=True,codes='tup') # Tell RPi latest condition of the motor.
        else:
            self.RPi.Write('tune rejected ' + self.MotorName + ' ' + IntToTimeString(self.LatestTuneTime) + ' ' + str(delta) + ': Motor not configured')
            self.LogFile.Log("error : TunePosition(" + self.MotorName + ") Rejected, motor is not yet configured.")

    def SetPins(self,stepBCM,directionBCM,enableBCM,faultBCM):
        """ Allocate pin numbers for the various GPIO pins required. """
        self.StepBCM = stepBCM # Pin(stepBCM, Pin.OUT, Pin.PULL_DOWN) # Set pin to OUTPUT. This sends the MOVE pulse to the controller.
        self.StepBCM.SetValue(False) # (0) # Turn pin off.
        self.tmc._pin_step = stepBCM
        self.LogFile.Log(self.MotorName, 'Step pin', self.StepBCM.PinNumber)
        
        self.DirectionBCM = directionBCM # Pin(directionBCM, Pin.OUT, Pin.PULL_DOWN) # Set pin to OUTPUT. This sets the MOVE direction to the controller.
        self.DirectionBCM.SetValue(False) # (0)  # Turn pin off.
        self.tmc._pin_dir = directionBCM
        self.LogFile.Log(self.MotorName, 'Direction pin', self.DirectionBCM.PinNumber)
        
        self.EnableBCM = enableBCM # Pin(enableBCM, Pin.OUT, Pin.PULL_DOWN) # Set pin to OUTPUT. This enables/disabled the motor.
        self.EnableBCM.SetValue(False) # (0)  # Turn pin off.
        self.tmc._pin_en = enableBCM
        
        self.FaultBCM = faultBCM # Pin(faultBCM, Pin.IN) # Set pin to INPUT. Will EARTH when triggered.


    def SetConfig(self,gearratio,motorstepsperrev,minangle,maxangle,restangle,currentangle,orientation,backlashangle,modesignals=[False,False,False]):
        """ Update the motor configuration based upon the configuration values received.
            Current position is calculated based upon the currentangle value received.
            - This is because the RPi knows the 'angle' when the system last ran,
              but the matching step position may not be the same if the motor configuration has changed since.
              So it is recalculated from the angle to ensure the previous and current positions represent the same physical angle. """
        self.GearRatio = gearratio
        self.MotorStepsPerRev = motorstepsperrev
        self.MinAngle = minangle
        self.MaxAngle = maxangle
        self.RestAngle = restangle
        self.Orientation = orientation
        self.BacklashAngle = backlashangle
        # Reapply dependent calculations.
        # Microstepratio and UseMicrostepping now obsolete parameters.
        self.WaitTime = self.SlowTime # The time between pulses, starts slow, reduces as a move progresses. Resets every time a new target is set.
        self.StepDir = 1 # The direction of a particular move +/-1. It always represents a SINGLE FULL STEP.
        self.LastStepDir = 0 # Record the 'last' direction that the motor moved in. This may be useful for handling gear backlash. Starts at ZERO (No direction)
        self.DriftSteps = 0 # This is the number of steps 'error' that DriftTracking has identified. It must be incorporated back into motor movements as smoothly as possible. Consider backlash etc.
        self.AxisStepsPerRev = self.MotorStepsPerRev * self.GearRatio
        # AngleToStep and StepToAngle only work from here on!
        self.RestPosition = self.AngleToStep(self.RestAngle)
        self.CurrentPosition = self.TargetPosition = self.AngleToStep(currentangle)
        self.MinPosition = self.AngleToStep(self.MinAngle) # Min clockwise movement. Location of limit switch in steps (This is self calibrating when in use).
        self.MaxPosition = self.AngleToStep(self.MaxAngle) # Max clockwise movement. Location of limit switch in steps (This is self calibrating when in use).
        self.RequestedPosition = self.CurrentPosition
        
        # Hard coded for now
        self.tmc.set_current(500)                   # Current 500mA
        self.tmc.set_max_speed_fullstep(500)        # Max speed (full steps/sec)
        self.tmc.set_speed_fullstep(400)            # Set speed
        self.tmc.set_acceleration_fullstep(500)     # Acceleration (full steps/sec/sec)

        return self.MotorConfigured

    def ConfigureMotor(self,line):
        """ This loads the motor configuration received from the RPi.
            It can override some default values in the configuration.
            All values are optional.
            Any value of 'none' is ignored.

            configure motor 20231016085541 azimuth 130.492 0 360 0.0 -1 0.001 0.05 0.003 10 n  n 90.0 240 400  1 180.0 nnn 1 n nnn
                0       1         2           3       4    5  6   7  8   9     10    11  12 13 14 15   16 17  18 19    20 21 22 23

                 2 = UTC timestamp when message sent.
                 3 = Motor name.
                 4 = Last reported (Current) angle.
                 5 = Minimum allowed angle.
                 6 = Maximum allowed angle.
                 7 = Backlash angle.
                 8 = Motor orientation.
                 9 = FastTime.
                10 = SlowTime.
                11 = TimeDelta.
                12 = Delay between automatic status messages.
                13 = FaultSensitive (stop if fault signal from driver).
                14 = OptimiseMoves (allows unlimited full rotation).
                15 = LimitAngle (motor will not cross this angle) - under development.
                16 = Gear ratio.
                17 = Motor steps per revolution (1 revolution of motor).
                18 = SlewStepMultiplier (number of steps taken with a SLEW move).
                19 = Motor rest angle (when homed).
                20 = Microstepping mode signals (used when making observation).
                21 = SlewMotor flag (Can motor make FULL STEP moves during large position changes). <- Experimental feature.
                22 = Slew stepping mode signals (used when making large position changes). <- Experimental feature.
            """
        try:
            lineitems = line.lower().split(' ')
            lc = len(lineitems)
            self.Clock.UpdateClockFromString(lineitems[2]) # Check that the clock is as synchronised as possible.
            if lineitems[7] != 'none':
                self.BacklashAngle = float(lineitems[7]) # Set new backlash angle for motor.
            if lineitems[8] != 'none':
                self.Orientation = int(lineitems[8]) # Set new orientation for motor.
            if lineitems[9] != 'none':
                self.FastTime = float(lineitems[9]) # Set new FASTEST PULSE time for motor.
            if lineitems[10] != 'none':
                self.SlowTime = float(lineitems[10]) # Set new SLOWEST PULSE time for motor.
            if lineitems[11] != 'none':
                self.TimeDelta = float(lineitems[11]) # Set new acceleration rate for motor.
            if self.WaitTime < self.FastTime: self.WaitTime = self.FastTime # Current motor speed cannot be faster than new fastest limit.
            if self.WaitTime > self.SlowTime: self.WaitTime = self.SlowTime # Current motor speed cannot be slower than new slowest limit.
            if lineitems[12] != 'none': # Must be within reasonable limits.
                temp = int(lineitems[12])
                if temp < 1: temp = 1
                elif temp > 30: temp = 30
                self.StatusTimer = timer(self.MotorName,temp) # Set new repeat time for sending motor status messages back to the RPi
            if lineitems[13] != 'none': # Enable/disable fault sensitivity. DRV8825 can then abort an observation.
                self.FaultSensitive = StringToBool(lineitems[13])
            if lineitems[14] != 'none': # Enable/disable move optimisation.
                self.OptimiseMoves = StringToBool(lineitems[14])
            if lineitems[16] != 'none': # Define GearRatio.
                self.GearRatio = float(lineitems[16])
            if lineitems[17] != 'none': # Define motorstepsperrev
                self.MotorStepsPerRev = int(lineitems[17])
                self.AxisStepsPerRev = self.MotorStepsPerRev * self.GearRatio
            self.SlewStepMultiplier = int(lineitems[18]) # Number of steps taken with a larget SLEW move (if microstepping in place).
            if lineitems[19] != 'none': # Define restangle
                self.RestAngle = float(lineitems[19])
            try:
                self.move_msteps = int(lineitems[20])
            except ValueError:
                self.LogFile.Log("Move microsteps invalid. Defaulting to 8")
                self.move_msteps = 8
            if lc > 21: # Allow SLEW fast moves.
                self.SlewMotor = StringToBool(lineitems[21]) # Are SLEW fast moves allowed?
            if lc > 22: # Load mode signals to select FULL STEPS when performing large fast slews.
                try:
                    self.slew_msteps = int(lineitems[22])
                except ValueError:
                    self.LogFile.Log("Slew microsteps invalid. Defaulting to 4")
                    self.slew_msteps = 4
            # Restore min/max/current/limit position only after the MotorStepsPerRev is known, if microstepping has changed above these will be different.
            if lineitems[4] != 'none': # To apply to live copy.
                self.CurrentPosition = self.AngleToStep(float(lineitems[4]))
            if lineitems[5] != 'none':
                self.MinAngle = float(lineitems[5]) # Set new minimum angle for motor.
                self.MinPosition = self.AngleToStep(self.MinAngle)
            if lineitems[6] != 'none':
                self.MaxAngle = float(lineitems[6]) # Set new maximum angle for motor.
                self.MaxPosition = self.AngleToStep(self.MaxAngle)
            # Define movement limit on the motor. The motor will reverse around a limit rather than crossing it.
            if lineitems[15] != 'none': # Set a movement limit.
                limitangle = float(lineitems[15])
                self.LimitPosition = self.AngleToStep(limitangle)
            else:
                self.LimitPosition = None
            self.MotorConfigured = True
        except Exception as e:
            self.LogFile.Log("steppermotor.ConfigureMotor(line) failed: " + str(e))
            print("steppermotor.ConfigureMotor() failed.")
            self.ExceptionCounter.Raise() # Increment exception count for the session.
        self.ReportMotorConfig() # Report the configuration back to the RPi.
        return self.MotorConfigured

    def StepMove(self,stepsize=1):
        """ Move the motor one full step. Target must be initialized before calling this.
            This is the 'fast' version of MoveFullStep. With logic rearranged between MoveMotor and MoveFullStep. """
        self.StatusLed.Task('move') # Flash status LED with motor specific colour.
        if self.MotorEnabled: # If we've disabled the motor, then perform everything except the move pulse.
            self.StepBCM.SetValue(True) # value(1)
            time.sleep(self.WaitTime)
            self.StepBCM.SetValue(False) # value(0)
            time.sleep(self.WaitTime)
        self.CurrentPosition = (self.CurrentPosition + (self.StepDir * stepsize)) % self.AxisStepsPerRev
        # Accelerate the motor.
        if self.WaitTime > self.FastTime: # We can still accelerate
            self.WaitTime = max(self.WaitTime - self.DeltaTime, self.FastTime)

    def InvertSteps(self,motorsteps):
        """ Given a number of steps to move, return the inverse move. """
        if motorsteps > 0: # Instead of going forward, go backward.
            inversemove = motorsteps - self.AxisStepsPerRev
        else: # Instead of going backward, go forward.
            inversemove = motorsteps + self.AxisStepsPerRev
        return inversemove

    def EfficiencyCheck(self,motorsteps):
        """ Check motorsteps are efficient.
            If the motor is taking the long route to the new position, revise it. """
        inversemove = motorsteps
        if abs(motorsteps) > int(self.AxisStepsPerRev / 2): # We're going the long way round.
            inversemove = self.InvertSteps(motorsteps)
            self.LogFile.Log("steppermotor.EfficiencyCheck(): Inefficient move:",self.CurrentPosition,"to",self.TargetPosition,",",motorsteps,"steps, suggest",inversemove)
            print("Inefficient move:",self.CurrentPosition,"to",self.TargetPosition,",",motorsteps,"steps, suggest",inversemove)
        return inversemove

    def MoveMotorFast(self):
        """ Move the motor to the new target position.
            If the SlewMotor parameter is True, then this can use FULL STEPS to speed things up in the middle of large moves.
            This moves the telescope faster when using very fine microstepping.
            This moves to a 'full step' boundary on the motor, then switches to FULL STEP movements until close to the target.
            When close to the target it reverts to microstepping for fine tuning.
            Target must be defined before calling this.
            Large moves can take some time, so UART communication is maintained during moves.
            The motor will generally take the shortest path to the target position.
            It may take the longer route under some circumstances. (LimitPosition must not be crossed etc.) """
        MotorSteps = self.TargetPosition - self.CurrentPosition # How many steps to take?
        if self.OptimiseMoves: # Allowed to find shorter paths!
            inversemove = self.EfficiencyCheck(MotorSteps) # Check if this is the most efficient move.
            if inversemove != MotorSteps: # We're changing direction for a short cut.
                MotorSteps = inversemove
                print("steppermotor.MoveMotorFast moving",MotorSteps,"steps after efficiency check.")
                self.StepDir *= -1
        else: # Must move as instructed, but can report if a shorter path exists.
            _ = self.EfficiencyCheck(MotorSteps) # Check if this is the most efficient move.
        if MotorSteps != 0:
            self.StatusLed.Task('move') # Flash status LED with motor specific colour.
            if abs(MotorSteps) > 100: # Large moves will reset the 'OnTarget' flag.
                self.OnTarget = False
        if self.FaultBCM.GetValue() == False: # DRV8825 'fault' pin is triggered.
            if self.FaultSensitive: # The fault matters.
                if not self.FaultDetected:
                    print("Setting FAULT status.")
                    self.FaultDetected = True
                    self.LogFile.Log("steppermotor.MoveMotorFast(", self.MotorName, ') DRV8825 fault - terminating.')
                return
            else: # The fault does not matter.
                if not self.FaultDetected: # Only report once.
                    self.LogFile.Log("steppermotor.MoveMotorFast(", self.MotorName, ') DRV8825 fault - ignored.')
                    print("Setting FAULT status.")
                    self.FaultDetected = True
        else: # No DRV8825 fault, clear any previous fault status.
            if self.FaultDetected:
                self.LogFile.Log("steppermotor.MoveMotorFast(", self.MotorName, ') DRV8825 fault - cleared.')
                self.FaultDetected = False # No fault.
        if abs(self.StepDir) != 1: # self.StepDir must be +1 or -1
            self.LogFile.Log('MoveMotorFast: ' + self.MotorName + ' StepDir " + str(self.StepDir) + " is invalid. Must be +/-1')
            return
        if (self.StepDir * self.Orientation) > 0:
            self.DirectionBCM.SetValue(True) # value(1) # Move motor forward.
        else:
            self.DirectionBCM.SetValue(False) # value(0) # Move motor backwards.
        if self.StepDir != self.LastStepDir: # We have a change of direction.
            self.LogFile.Log('MoveMotorFast: ' + self.MotorName + ' changed direction (' + str(self.StepDir) + ' vs ' + str(self.LastStepDir) + '). Backlash?')
        self.LastStepDir = self.StepDir # Record the direction that the motor is moving in. This may be useful for handling gear backlash etc.
        if self.SlewMotor: # We're allowed to make FAST moves using FULL STEPS if there's a long way to go.
            targettolerance = 100 * self.SlewStepMultiplier # This is as close as we want to get with large slew moves. Finetuning is done with MoveMotor() afterwards.
            # Set microstepping mode.
            self.tmc.set_microstepping_resolution(self.move_msteps)
            self.LogFile.Log('MoveMotorFast: Setting microstepping to '+str(self.move_msteps)+'.')
            self.WaitTime = self.SlowTime # Start with slow move pulses. This reduces each time we call StepMove().
            while MotorSteps != 0:
                if self.CurrentPosition % self.SlewStepMultiplier == 0: # On FULLSTEP boundary. Switch to FULL STEPS.
                    break
                MotorSteps = MotorSteps - self.StepDir # REDUCE (-ve) the number of steps to take.
                self.StepMove(stepsize=1) # This will update CurrentPosition on-the-fly as the motor moves.
                self.SendMotorStatus(codes='mov') # Long slow moves would cause RPi to trigger a reset and the user won't see progress until the end, so send regular status updates.
                for i in range(1): # Check UART buffers. Can loop multiple times if you want to, but it pauses movement while checking.
                    self.RPi.BufferInput() # Keep polling for input from the RPi.
                    self.RPi.WritePoll() # Keep sending data to RPi.
            # Set slew (full stepping) mode.
            self.tmc.set_microstepping_resolution(self.slew_msteps)
            self.LogFile.Log('MoveMotorFast: Setting microstepping to '+str(self.slew_msteps)+' (slew).')

            self.WaitTime = self.SlowTime # Start with slow move pulses. This reduces each time we call StepMove().
            while MotorSteps != 0:
                if abs(self.TargetPosition - self.CurrentPosition) <= targettolerance: # We've got close with FULLSTEPS, switch back to microsteps.
                    break
                MotorSteps = (MotorSteps - (self.StepDir * self.SlewStepMultiplier)) # REDUCE (-ve) the number of steps to take.
                self.StepMove(stepsize=self.SlewStepMultiplier) # This will update CurrentPosition on-the-fly as the motor moves.
                self.SendMotorStatus(codes='mov') # Long slow moves would cause RPi to trigger a reset and the user won't see progress until the end, so send regular status updates.
                for i in range(1): # Check UART buffers. Can loop multiple times if you want to, but it pauses movement while checking.
                    self.RPi.BufferInput() # Keep polling for input from the RPi.
                    self.RPi.WritePoll() # Keep sending data to RPi.
        # Set microstepping mode. Regardless of 'slew' mode or not, now use configured microstepping to complete the move.
        self.tmc.set_microstepping_resolution(self.move_msteps)
        self.LogFile.Log('MoveMotorFast: Setting microstepping to '+str(self.move_msteps)+'.')

        self.WaitTime = self.SlowTime # Start with slow move pulses. This reduces each time we call StepMove().
        while MotorSteps != 0:
            MotorSteps = MotorSteps - self.StepDir # REDUCE (-ve) the number of steps to take.
            self.StepMove(stepsize=1) # This will update CurrentPosition on-the-fly as the motor moves.
            self.SendMotorStatus(codes='mov') # Long slow moves would cause RPi to trigger a reset and the user won't see progress until the end, so send regular status updates.
            for i in range(1): # Check UART buffers. Can loop multiple times if you want to, but it pauses movement while checking.
                self.RPi.BufferInput() # Keep polling for input from the RPi.
                self.RPi.WritePoll() # Keep sending data to RPi.
        self.CheckOnTarget() # Are we actually pointing at the target?
        if self.CurrentPosition != self.TargetPosition: # Did the motor slew close to the intended position? (May not be the requested target if movement limits reached)
            self.LogFile.Log("MoveMotorFast(" + self.MotorName + "): End. CurrentPosition (" + str(self.CurrentPosition) + ") is NOT TargetPosition (" + str(self.TargetPosition) + ")!")
        self.StatusLed.Task('idle')

    def callback(self):
        """ Called once a step, so we can update here. """
        self.CurrentPosition = self.tmc._current_pos % self.AxisStepsPerRev
        self.SendMotorStatus(codes='mov')
        self.RPi.BufferInput() # Keep polling for input from the RPi.
        self.RPi.WritePoll() 
                        
    def MoveMotor(self):
        #########################################################
        # Deal with self.MotorEnabled and the DIAG(Fault) Pin 
        # Consider Slew mode
        #########################################################
        """ Moving using the TMC driver 'run_to_position_steps'. """
        MotorSteps = self.TargetPosition - self.CurrentPosition
        
        if MotorSteps != 0:
            self.StatusLed.Task('move') # Flash status LED with motor specific colour.
            if abs(MotorSteps) > 100:   # Large moves will reset the 'OnTarget' flag.
                self.OnTarget = False
         
            if abs(self.StepDir) != 1: # self.StepDir must be +1 or -1
                self.LogFile.Log('MoveMotor: ' + self.MotorName + ' StepDir " + str(self.StepDir) + " is invalid. Must be +/-1')
                return      
            
            if (self.StepDir * self.Orientation) > 0:
                self.tmc.set_direction_pin_or_reg(True) # value(1) # Move motor forward.
            else:
                self.tmc.set_direction_pin_or_reg(False) # value(0) # Move motor backwards.
            if self.StepDir != self.LastStepDir: # We have a change of direction.
                self.LogFile.Log('MoveMotor: ' + self.MotorName + ' changed direction (' + str(self.StepDir) + ' vs ' + str(self.LastStepDir) + '). Backlash?')
                self.LastStepDir = self.StepDir # Record the direction that the motor is moving in. This may be useful for handling gear backlash etc.
            
            # Set microstepping mode if needed
            if(self.tmc.get_microstepping_resolution() != self.move_msteps):
                self.tmc.set_microstepping_resolution(self.move_msteps)
            
            # Move  
            self.tmc._current_pos = self.CurrentPosition            
            self.tmc.run_to_position_steps(MotorSteps, callback = self.callback)
            self.CurrentPosition = self.tmc._current_pos
            
            # Check 
            self.CheckOnTarget() # Are we actually pointing at the target?
            if self.CurrentPosition != self.TargetPosition: # Did the motor reach intended position? (May not be the requested target if movement limits reached)
                self.LogFile.Log("MoveMotor(" + self.MotorName + "): End. CurrentPosition (" + str(self.CurrentPosition) + ") is NOT TargetPosition (" + str(self.TargetPosition) + ")!")
            
            self.StatusLed.Task('idle')
        else:
            self.LogFile.Log("MoveMotor(" + self.MotorName + "): No steps to move")

    def StepToAngle(self, steps=None):
        """ Convert a number of steps to a final angle (0-360) of movement. """
        if steps != None:
            result = steps * 360.0 / float(self.AxisStepsPerRev)
        else:
            result = None
        return result

    def AngleToStep(self, deg=None):
        """ Convert a final angle of movement to the nearest whole number of motor steps. """
        if deg != None:
            result = int(round(deg * float(self.AxisStepsPerRev) / 360,0))
        else:
            result = None
        return result

    def ReportMotorConfig(self):
        """ Report motor configuration back to the RPi. """
        line = "# Motor " + self.MotorName + " conf 1: "
        line += IntToTimeString(self.Clock.Now()) + " "
        line += "MinA " + str(self.MinAngle) + " "
        line += "MinP " + str(self.MinPosition) + " "
        line += "MaxA " + str(self.MaxAngle) + " "
        line += "MaxP " + str(self.MaxPosition) + " "
        line += "LimP " + str(self.LimitPosition) + " "
        line += "RestA " + str(self.RestAngle) + " "
        self.RPi.Write(line) # Send over UART to RPi.
        line = "# Motor " + self.MotorName + " conf 2: "
        line += "FastT " + str(self.FastTime) + " "
        line += "SlowT " + str(self.SlowTime) + " "
        line += "TDelta " + str(self.TimeDelta) + " "
        line += "FaultS " + str(self.FaultSensitive) + " "
        line += "BackA " + str(self.BacklashAngle) + " "
        line += "Orient " + str(self.Orientation) + " "
        line += "OptMvs " + str(self.OptimiseMoves) + " "
        self.RPi.Write(line) # Send over UART to RPi.
        line = "# Motor " + self.MotorName + " conf 3: "
        line += "GearRat " + str(self.GearRatio) + " "
        line += "uS/Rev " + str(self.MotorStepsPerRev) + " "
        line += "usMode " + "None" # Microstepping mode pin settings.
        line += "AxStp/Rev " + str(int(self.AxisStepsPerRev)) + " "
        line += "MtrCnf " + str(self.MotorConfigured) + " "
        self.RPi.Write(line) # Send over UART to RPi.

    def SendMotorStatus(self,immediate=False,codes='?-?'):
        """ Generate status message to RPi.
            The RPi uses this to decide what commands and configurations to send to the microcontroller.
            This can be triggered via multiple methods and in some circumstances can flood the RPi with
            messages. So there is a maximum repeat rate built in.
            immediate: True means that the status is sent even if not due.
                       False means that the status is only sent if the timer is due.
            codes: Optional string of codes that are added to the status message. (Debug/test/dev etc)
            """
        if immediate or self.StatusTimer.Due(): # Only send the status at regular intervals, otherwise we flood communications.
            if self.SendStatus == False: # Status message is currently disabled. Inform that we're not sending it.
                self.RPi.Write('# SendMotorStatus ' + IntToTimeString(self.Clock.Now()) + ' ' + self.MotorName + ' disabled. ' + str(codes))
                print("SendMotorStatus",self.MotorName,"disabled.",codes)
                return
            line = 'motor status '
            line += IntToTimeString(self.Clock.Now()) + ' ' # Current local timestamp.
            line += self.MotorName + ' '
            line += BoolToString(self.Trajectory.Valid) + ' ' # TrajectoryValid
            line += IntToTimeString(self.Trajectory.ValidUntil()) + ' ' # When does the trajectory run out?
            line += str(len(self.Trajectory.TrajectoryList)) + ' ' # How many segments in the trajectory?
            line += str(self.CurrentPosition) + ' ' # Where is the camera at the moment?
            line += str(self.CurrentDegrees()) + ' ' # Where is the camera at the moment?
            line += BoolToString(self.MotorConfigured) + ' ' # MotorConfigured
            line += BoolToString(self.OnTarget) + ' ' # Motor is on target or not.
            line += str(self.WaitTime * 2) + ' ' # The pulse period (indicates speed) of the motor.
            line += str(self.VMot()) + ' ' # ADC0 is measuring motor voltage. Send the current ADC value.
            line += str(codes) + ' ' # Optional codes added to status message.
            self.RPi.Write(line) # Send over UART to RPi.
            # Reset the status timer.
            self.StatusTimer.Reset() # We've sent the regular status message, decide when the next is due.

