# Based on https://github.com/Short-bus/pilomar

VERSION = '1.1.0' # Software version reported to the RPi.
ACCEPTABLERPIVERSIONS = ['1.0', '1.1'] # Which RPi versions are acceptable? (Ignore patch level)

import time
import microcontroller
import board
import busio
import time
import digitalio
import gc

from pilomar.helpers import *
from pilomar.uart import *
from pilomar.session import *
from pilomar.stepper import *
from pilomar.process import *

DegreeSymbol = 'deg'

#-----------------------------------------------------------------------------------------------

(CircuitPythonVersion, Bootline) = check_version()
print("CircuitPython version:",CircuitPythonVersion,"environment:",Bootline)

# Over clock the TINY a bit :)
microcontroller.cpu.frequency = 200000000 # Set to 200MHz

# Status LED
StatusLed = statusled()
StatusLed.Task('init')
time.sleep(1)

# Exception Counter
ExceptionCounter = exceptioncounter(StatusLed)
print("Base clock:",IntToTimeString(time.time()))

# Session
SessionTimer = timer('session',30,offset=7)

# Logger
LogFile = logfile()
print ('ResetReason: ' + str(microcontroller.cpu.reset_reason))

# Pi Host and Communication
RPi = uarthost(LogFile, ExceptionCounter, StatusLed, version=VERSION)
LogFile.setHost(RPi)

# Clock
Clock = clock(LogFile, ExceptionCounter, RPi, TimeValue=time.time()) # Simulate RTC
LogFile.Clock = Clock # Tell the LogFile which clock to use.
RPi.Clock = Clock

# Motors
Azimuth = steppermotor('azimuth', LogFile, ExceptionCounter, StatusLed, Clock, RPi, id=0)
Azimuth.SetPins(stepBCM=AzimuthStepBCM, directionBCM=CommonDirectionBCM, enableBCM=CommonEnableBCM, faultBCM=AzimuthFaultBCM)
Azimuth.SetConfig(gearratio=(60*4),motorstepsperrev=400,minangle=45.0,maxangle=315.0,restangle=180.0,currentangle=180.0,orientation=1,backlashangle=0.0)

Altitude = steppermotor('altitude', LogFile, ExceptionCounter, StatusLed, Clock, RPi, id=1)
Altitude.SetPins(stepBCM=AltitudeStepBCM, directionBCM=CommonDirectionBCM, enableBCM=CommonEnableBCM, faultBCM=AltitudeFaultBCM)
Altitude.SetConfig(gearratio=(60*4),motorstepsperrev=400,minangle=0.0,maxangle=90.0,restangle=0.0,currentangle=0.0,orientation=-1,backlashangle=0.0)

Motors = [Azimuth, Altitude] # Control over 'all' motors.

# Session related
Session = picosession(LogFile, ExceptionCounter, Clock, Motors, RPi, CircuitPythonVersion)
Process = process(LogFile, ExceptionCounter, StatusLed, RPi, Session, Clock, Motors)
MemMgr = memorymanager()

#-----------------------------------------------------------------------------------------------
# Here we go
#-----------------------------------------------------------------------------------------------
print ('Starting...')
RPi.Reset()
line = "defined motors "+" ".join([i.MotorName for i in Motors])
RPi.Write(line)

try:
    while True: # Full interaction

        try:
            LogFile.SendCheck() # Keep log file buffer under control. Flushes the buffer if it gets too large.
        except Exception as e:
            LogFile.Log("Main:LogFile.SendCheck failed.",e)
            ExceptionCounter.Raise() # Increment exception count for the session.

        line = ''
        try:
            line = RPi.Read() # Any input from the Raspberry Pi in the cache?
        except Exception as e:
            LogFile.Log("Main:RPi.Read failed.",e)
            ExceptionCounter.Raise() # Increment exception count for the session.

        try:
            if len(line) != 0: Process.ProcessInput(line) # Process it.
        except Exception as e:
            LogFile.Log("Main:ProcessInput failed.",e)
            ExceptionCounter.Raise() # Increment exception count for the session.

        try:
            Session.TrajectorySafety() # If no recent receipt from RPi, assume comms break take precautions... Clear trajectories?
        except Exception as e:
            LogFile.Log("Main: SessionTrajectorySafety() failed.",e)
            ExceptionCounter.Raise() # Increment exception count for the session.

        if Session.Quit: break

        try:
            if SessionTimer.Due():
                Session.SendSessionStatus(codes='tmr') # Send session status messages.
                Session.SendCpuStatus() # Send microcontroller status.
        except Exception as e:
            LogFile.Log("Main: SessionTimer failed.",e)
            ExceptionCounter.Raise() # Increment exception count for the session.

        try:
            Session.SendMotorStatus('azimuth',codes='tmr') # Send azimuth status message.
        except Exception as e:
            LogFile.Log("Main: Azimuth status failed.",e)
            ExceptionCounter.Raise() # Increment exception count for the session.

        try:
            Session.SendMotorStatus('altitude',codes='tmr') # Send altitude status message.
        except Exception as e:
            LogFile.Log("Main: Altitude status failed.",e)
            ExceptionCounter.Raise() # Increment exception count for the session.

        try:
            RPi.WritePoll() # Send anything in the transmit buffer if it's safe.
        except Exception as e:
            LogFile.Log("Main: RPi.WritePoll() failed.",e)
            ExceptionCounter.Raise() # Increment exception count for the session.

        try:
            Session.AutoMoveMotors() # Move motors if allowed to.
        except Exception as e:
            LogFile.Log("Main: AutoMoveMotors failed.",e)
            ExceptionCounter.Raise() # Increment exception count for the session.

        try:
            MemMgr.Poll() # Check memory condition.
        except Exception as e:
            LogFile.Log("Main: MemMgr.Poll() failed.",e)
            ExceptionCounter.Raise() # Increment exception count for the session.

except Exception as e:
        print('Mainloop failed:', str(e))
        StatusLed.Task('error')
        print(e.args)
        ExceptionCounter.Raise() # Increment exception count for the session.

#-----------------------------------------------------------------------------------------------
# Shutdown procedure...
#-----------------------------------------------------------------------------------------------

RPi.Write('controller stopping')
print ('controller stopping...')
LogFile.SendCheck(force=True)
RPi.Write('# GCCount ' + str(MemMgr.GCCount))
RPi.Write('controller stopped')

LoopCounter = 0
print ('Flushing final comms to RPi.')
print ('Further input from RPi will be ignored.')

while len(RPi.WriteQueue) > 0:
    RPi.WritePoll()
    LoopCounter += 1
    if LoopCounter > 1000:
        print ('Flushing incomplete.')
        break
print ('controller stopped')
