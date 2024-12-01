# Based on https://github.com/Short-bus/pilomar

import time
import microcontroller
import board
import busio
import time
import digitalio
import gc

VERSION = '1.1.0' # Software version reported to the RPi.
ACCEPTABLERPIVERSIONS = ['1.0', '1.1'] # Which RPi versions are acceptable? (Ignore patch level)
FEATURES = ['RP2040','VMOT','TINY']
#DRIVER = 'DRV8825'
DRIVER = 'TMC2209'

#-----------------------------------------------------------------------------------------------
exec(open("./pilomar/helpers.py").read())

(CircuitPythonVersion, Bootline) = check_version()
print("CircuitPython version:",CircuitPythonVersion,"environment:",Bootline)

StatusLed = statusled()
StatusLed.Task('init')
time.sleep(1)

ExceptionCounter = exceptioncounter()
DegreeSymbol = 'deg'
print("Base clock:",IntToTimeString(time.time()))

SessionTimer = timer('session',30,offset=7)
LogFile = logfile()
print ('ResetReason: ' + str(microcontroller.cpu.reset_reason))

#-----------------------------------------------------------------------------------------------
exec(open("./pilomar/uart.py").read())
RPi = uarthost(channel=0)

Clock = clock(time.time()) # Simulate RTC
LogFile.Clock = Clock # Tell the LogFile which clock to use.

#-----------------------------------------------------------------------------------------------
exec(open("./pilomar/trajectory.py").read())
exec(open("./pilomar/stepper"+DRIVER+".py").read()) # Note this file contains the PIN assignments for motors !

# Configure Motors.
Azimuth = steppermotor('azimuth', id=0)
Azimuth.SetPins(stepBCM=AzimuthStepBCM,directionBCM=CommonDirectionBCM,
                mode0BCM=CommonMode0BCM,mode1BCM=CommonMode1BCM,mode2BCM=CommonMode2BCM,
                enableBCM=CommonEnableBCM,faultBCM=AzimuthFaultBCM) # Direct control over Azimuth motor.
Azimuth.SetConfig(gearratio=(60 * 4),motorstepsperrev=400,minangle=45.0,maxangle=315.0,restangle=180.0,currentangle=180.0,orientation=1,backlashangle=0.0)

Altitude = steppermotor('altitude', id=1)
Altitude.SetPins(stepBCM=AltitudeStepBCM,directionBCM=CommonDirectionBCM,
                 mode0BCM=CommonMode0BCM,mode1BCM=CommonMode1BCM,mode2BCM=CommonMode2BCM,
                 enableBCM=CommonEnableBCM,faultBCM=AltitudeFaultBCM) # Direct control over Altitude motor.
Altitude.SetConfig(gearratio=(60 * 4),motorstepsperrev=400,minangle=0.0,maxangle=90.0,restangle=0.0,currentangle=0.0,orientation=-1,backlashangle=0.0)

Motors = [Azimuth, Altitude] # Control over 'all' motors.

#-----------------------------------------------------------------------------------------------
exec(open("./pilomar/session.py").read())
Session = picosession()
MemMgr = memorymanager()

exec(open("./pilomar/process.py").read())
#-----------------------------------------------------------------------------------------------
print ('Starting...')
RPi.Reset()
line = "defined motors "+" ".join([i.MotorName for i in Motors])
RPi.Write(line)

# This is the main processing loop.
try:
    while True: # Full interaction

        try:
            LogFile.SendCheck() # Keep log file buffer under control. Flushes the buffer if it gets too large.
        except Exception as e:
            LogFile.Log("Main:LogFile.SendCheck failed.",e)
            print("Main:LogFile.SendCheck failed.",e)
            ExceptionCounter.Raise() # Increment exception count for the session.

        line = ''
        try:
            line = RPi.Read() # Any input from the Raspberry Pi in the cache?
        except Exception as e:
            LogFile.Log("Main:RPi.Read failed.",e)
            print("Main:RPi.Read failed.",e)
            print("Main:Failed on",line)
            ExceptionCounter.Raise() # Increment exception count for the session.
        try:
            if len(line) != 0: ProcessInput(line) # Process it.
        except Exception as e:
            LogFile.Log("Main:ProcessInput failed.",e)
            print("Main:ProcessInput failed.",e)
            print("Main:Failed on",line)
            ExceptionCounter.Raise() # Increment exception count for the session.

        if 'TINY' in FEATURES and BootBCM.Released():
            LogFile.Log("Main:BootBCM user key pressed.")
            print("Main:BootBCM user key pressed.")

        try:
            Session.TrajectorySafety() # If no recent receipt from RPi, assume comms break take precautions... Clear trajectories?
        except Exception as e:
            LogFile.Log("Main: SessionTrajectorySafety() failed.",e)
            print("Main: SessionTrajectorySafety() failed.",e)
            ExceptionCounter.Raise() # Increment exception count for the session.

        if Session.Quit: break

        try:
            if SessionTimer.Due():
                Session.SendSessionStatus(codes='tmr') # Send session status messages.
                if 'RP2350' in FEATURES: SendCpuStatus() # Send microcontroller status.
        except Exception as e:
            LogFile.Log("Main: SessionTimer failed.",e)
            print("Main: SessionTimer failed.",e)
            ExceptionCounter.Raise() # Increment exception count for the session.

        try:
            Session.SendMotorStatus('azimuth',codes='tmr') # Send azimuth status message.
        except Exception as e:
            LogFile.Log("Main: Azimuth status failed.",e)
            print("Main: Azimuth status failed.",e)
            ExceptionCounter.Raise() # Increment exception count for the session.

        try:
            Session.SendMotorStatus('altitude',codes='tmr') # Send altitude status message.
        except Exception as e:
            LogFile.Log("Main: Altitude status failed.",e)
            print("Main: Altitude status failed.",e)
            ExceptionCounter.Raise() # Increment exception count for the session.

        try:
            RPi.WritePoll() # Send anything in the transmit buffer if it's safe.
        except Exception as e:
            LogFile.Log("Main: RPi.WritePoll() failed.",e)
            print("Main: RPi.WritePoll() failed.",e)
            ExceptionCounter.Raise() # Increment exception count for the session.

        try:
            Session.AutoMoveMotors() # Move motors if allowed to.
        except Exception as e:
            LogFile.Log("Main: AutoMoveMotors failed.",e)
            print("Main: AutoMoveMotors failed.",e)
            ExceptionCounter.Raise() # Increment exception count for the session.

        try:
            MemMgr.Poll() # Check memory condition.
        except Exception as e:
            LogFile.Log("Main: MemMgr.Poll() failed.",e)
            print("Main:MemMgr.Poll() failed.",e)
            ExceptionCounter.Raise() # Increment exception count for the session.

except Exception as e:
        print('Mainloop failed:', str(e))
        StatusLed.Task('error')
        print(e.args)
        ExceptionCounter.Raise() # Increment exception count for the session.

# Shutdown procedure...
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
