import board
import digitalio
import time

#----------------------------------------------------------------------------------------------------------

class steppermotor():
    def __init__(self,name):
        self.MotorName = name
        self.DriverType = 'tmc2209'
        self.FastTime = 0.0003     # The fastest pulse time for moving the motor.
        self.SlowTime = 0.001      # The slowest pulse time for moving the motor.
        self.DeltaTime = 0.001     # The acceleration amount moving from SlowTime to FastTime as the motor gets going.
        self.StepDir = 1           # The direction of a particular move +/-1. It always represents a SINGLE FULL STEP
        self.AxisStepsPerRev = 400 # This is without gearing

        self.CurrentPosition = 0
        self.TargetPosition = 0
        self.CurrentAngle = 0
        self.TargetAngle = 0

        self.MicroSteps = 8

        # Pins
        self.Disable = digitalio.DigitalInOut(board.GP2)
        self.Disable.direction = digitalio.Direction.OUTPUT
        self.StepBCM = digitalio.DigitalInOut(board.GP29)
        self.StepBCM.direction = digitalio.Direction.OUTPUT
        self.DirectionBCM = digitalio.DigitalInOut(board.GP3)
        self.DirectionBCM.direction = digitalio.Direction.OUTPUT

        # Motor On
        self.Disable.value = False

    def MoveMotor(self):
        MotorSteps = self.TargetPosition - self.CurrentPosition  # Steps we need to take
        self.WaitTime = self.SlowTime # Start with slow move pulses. This reduces each time we call MoveFullStep().

        while MotorSteps != 0:
            MotorSteps = MotorSteps - self.StepDir # REDUCE (-ve) the number of steps to take.
            self.MoveFullStep(stepsize=1) # This will update CurrentPosition on-the-fly as the motor moves.

    def MoveFullStep(self,stepsize=1):
        if (self.StepDir) > 0:
            self.DirectionBCM.value = True # value(1) # Move motor forward.
        else:
            self.DirectionBCM.value = False # value(0) # Move motor backwards.

        self.StepBCM.value = True
        time.sleep(self.WaitTime)
        self.StepBCM.value = False
        time.sleep(self.WaitTime)
        self.CurrentPosition += (self.StepDir * stepsize)

        # Accelerate the motor.
        if self.WaitTime > self.FastTime: # We can still accelerate
            self.WaitTime = max(self.WaitTime - self.DeltaTime, self.FastTime)

    def AngleToStep(self, deg):
        result = int(round(deg * float(self.AxisStepsPerRev) / 360))
        return self.MicroSteps * result

    def SetNewTargetAngle(self,newangle):
        self.TargetAngle = newangle
        self.TargetPosition = self.AngleToStep(newangle) # Convert it into the nearest absolute STEP position.

        if self.ChangeSteps() > 0:
            self.StepDir = 1 # Which direction do we move in?
        else:
            self.StepDir = -1

    def ChangeSteps(self):
        return (self.TargetPosition - self.CurrentPosition)

    def GoToAngle(self,newangle):
        self.SetNewTargetAngle(newangle)
        self.MoveMotor()
        self.CurrentAngle = self.TargetAngle

#----------------------------------------------------------------------------------------------------------
# Main program

motor=steppermotor('Stepper')

while True:
    print("Current position ", motor.CurrentAngle," degrees")
    angle=float(input("Enter target angle: "))
    motor.GoToAngle(angle)
