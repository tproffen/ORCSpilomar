import time
import board
import busio
import digitalio
import math
import struct

from pilomar.enum import *

#-----------------------------------------------------------------------------------------------
class reg(Enum):
    #addresses
    GCONF           =   0x00
    GSTAT           =   0x01
    IFCNT           =   0x02
    IOIN            =   0x06
    IHOLD_IRUN      =   0x10
    TSTEP           =   0x12
    VACTUAL         =   0x22
    TCOOLTHRS       =   0x14
    SGTHRS          =   0x40
    SG_RESULT       =   0x41
    MSCNT           =   0x6A
    CHOPCONF        =   0x6C
    DRVSTATUS       =   0x6F

    #GCONF
    i_scale_analog      = 1<<0
    internal_rsense     = 1<<1
    en_spreadcycle      = 1<<2
    shaft               = 1<<3
    index_otpw          = 1<<4
    index_step          = 1<<5
    pdn_disable         = 1<<6
    mstep_reg_select    = 1<<7

    #GSTAT
    reset               = 1<<0
    drv_err             = 1<<1
    uv_cp               = 1<<2

    #CHOPCONF
    toff0               = 1<<0
    toff1               = 1<<1
    toff2               = 1<<2
    toff3               = 1<<3
    vsense              = 1<<17
    msres0              = 1<<24
    msres1              = 1<<25
    msres2              = 1<<26
    msres3              = 1<<27
    intpol              = 1<<28

    #IOIN
    io_enn              = 1<<0
    io_step             = 1<<7
    io_spread           = 1<<8
    io_dir              = 1<<9

    #DRVSTATUS
    stst                = 1<<31
    stealth             = 1<<30
    cs_actual           = 31<<16
    t157                = 1<<11
    t150                = 1<<10
    t143                = 1<<9
    t120                = 1<<8
    olb                 = 1<<7
    ola                 = 1<<6
    s2vsb               = 1<<5
    s2vsa               = 1<<4
    s2gb                = 1<<3
    s2ga                = 1<<2
    ot                  = 1<<1
    otpw                = 1<<0

    #IHOLD_IRUN
    ihold               = 31<<0
    irun                = 31<<8
    iholddelay          = 15<<16

    #SGTHRS
    sgthrs              = 255<<0

    #others
    mres_256 = 0
    mres_128 = 1
    mres_64 = 2
    mres_32 = 3
    mres_16 = 4
    mres_8 = 5
    mres_4 = 6
    mres_2 = 7
    mres_1 = 8

#-----------------------------------------------------------------------------------------------
class Direction(Enum):
    """movement direction of the motor"""
    CCW = 0
    CW = 1

class MovementAbsRel(Enum):
    """movement absolute or relative"""
    ABSOLUTE = 0
    RELATIVE = 1

class MovementPhase(Enum):
    """movement phase"""
    STANDSTILL = 0
    ACCELERATING = 1
    MAXSPEED = 2
    DECELERATING = 3

class StopMode(Enum):
    """stopmode"""
    NO = 0
    SOFTSTOP = 1
    HARDSTOP = 2

#-----------------------------------------------------------------------------------------------
# Main Stepper motor class for TMC2209 (UART)
#-----------------------------------------------------------------------------------------------
class TMCStepper():

    mtr_id = 0
    ser = None
    r_frame  = [0x55, 0, 0, 0  ]
    w_frame  = [0x55, 0, 0, 0 , 0, 0, 0, 0 ]
    communication_pause = 0
    error_handler_running = False

    _msres = -1
    _steps_per_rev = 0
    _fullsteps_per_rev = 400
    _direction = True
    _stop = StopMode.NO
    _starttime = 0
    _sg_callback = None

    _msres = -1
    _steps_per_rev = 0
    _fullsteps_per_rev = 400

    _current_pos = 0                # current position of stepper in steps
    _target_pos = 0                 # the target position in steps
    _speed = 0.0                    # the current speed in steps per second
    _max_speed = 1.0                # the maximum speed in steps per second
    _max_speed_homing = 200         # the maximum speed in steps per second for homing
    _acceleration = 1.0             # the acceleration in steps per second per second
    _acceleration_homing = 10000    # the acceleration in steps per second per second for homing
    _sqrt_twoa = 1.0                # Precomputed sqrt(2*_acceleration)
    _step_interval = 0              # the current interval between two steps
    _min_pulse_width = 1            # minimum allowed pulse with in microseconds
    _last_step_time = 0             # The last step time in microseconds
    _n = 0                          # step counter
    _c0 = 0                         # Initial step size in microseconds
    _cn = 0                         # Last step size in microseconds
    _cmin = 0                       # Min step size in microseconds based on maxSpeed
    _sg_threshold = 100             # threshold for stallguard
    
    _movement_abs_rel = MovementAbsRel.ABSOLUTE
    _movement_phase = MovementPhase.STANDSTILL

#-----------------------------------------------------------------------------------------------
    def __init__(self, uart, LogFile, ExceptionCounter, 
                 mtr_id = 0, communication_pause = 0.004, pin_step=-1, pin_dir=-1, pin_en=-1):

        self.mtr_id = mtr_id
        self.LogFile = LogFile
        self.ExceptionCounter = ExceptionCounter
        self.communication_pause = communication_pause
        self.uart = uart
        self._pin_step = pin_step
        self._pin_dir = pin_dir
        self._pin_en = pin_en
        
        self.read_drv_status()  # Put drive status in Log

#-----------------------------------------------------------------------------------------------
    def compute_crc8_atm(self, datagram, initial_value=0):
        crc = initial_value
        # Iterate bytes in data
        for byte in datagram:
            # Iterate bits in byte
            for _ in range(0, 8):
                if (crc >> 7) ^ (byte & 0x01):
                    crc = ((crc << 1) ^ 0x07) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
                # Shift to next bit
                byte = byte >> 1
        return crc

#-----------------------------------------------------------------------------------------------
    def read_reg(self, register):
        self.uart.reset_input_buffer()

        self.r_frame[1] = self.mtr_id
        self.r_frame[2] = register
        self.r_frame[3] = self.compute_crc8_atm(self.r_frame[:-1])

        rtn = self.uart.write(bytearray(self.r_frame))
        if rtn != len(self.r_frame):
            self.LogFile.Log("TMCStepper("+self.mtr_id+"): Error in TMC2209 register read")
            self.ExceptionCounter.Raise()
            return False

        time.sleep(self.communication_pause)
        rtn = self.uart.read(12)
        time.sleep(self.communication_pause)

        return rtn

#-----------------------------------------------------------------------------------------------
    def read_int(self, register, tries=10):
        while True:
            tries -= 1
            rtn = self.read_reg(register)
            rtn_data = rtn[7:11]
            not_zero_count = len([elem for elem in rtn if elem != 0])
            if(len(rtn)<12 or not_zero_count == 0):
                self.LogFile.Log(f"""TMCStepper({self.mtr_id}):UART Communication Error: {len(rtn_data)} data bytes | {len(rtn)} total bytes""")
                self.ExceptionCounter.Raise()
            elif rtn[11] != self.compute_crc8_atm(rtn[4:11]):
                self.LogFile.Log("TMCStepper("+self.mtr_id+"): UART Communication Error: CRC MISMATCH")
                self.ExceptionCounter.Raise()
            else:
                break

            if tries<=0:
                self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): After 10 tries not valid answer")
                self.handle_error()
                return -1

        val = struct.unpack(">i",rtn_data)[0]
        return val

#-----------------------------------------------------------------------------------------------
    def write_reg(self, register, val):
        self.uart.reset_input_buffer()

        self.w_frame[1] = self.mtr_id
        self.w_frame[2] =  register | 0x80  # set write bit
        self.w_frame[3] = 0xFF & (val>>24)
        self.w_frame[4] = 0xFF & (val>>16)
        self.w_frame[5] = 0xFF & (val>>8)
        self.w_frame[6] = 0xFF & val
        self.w_frame[7] = self.compute_crc8_atm(self.w_frame[:-1])

        rtn = self.uart.write(bytes(self.w_frame))
        if rtn != len(self.w_frame):
            self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): Error in writing to UART register")
            self.ExceptionCounter.Raise()
            return False
        time.sleep(self.communication_pause)
        return True

#-----------------------------------------------------------------------------------------------
    def write_reg_check(self, register, val, tries=10):
        ifcnt1 = self.read_int(reg.IFCNT)

        if ifcnt1 == 255:
            ifcnt1 = -1

        while True:
            self.write_reg(register, val)
            tries -= 1
            ifcnt2 = self.read_int(reg.IFCNT)
            if ifcnt1 >= ifcnt2:
                self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): Writing to UART register failed")
                self.ExceptionCounter.Raise()
            else:
                return True
            if tries<=0:
                self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): After 10 tries no valid write access")
                self.handle_error()
                return -1

#-----------------------------------------------------------------------------------------------
    def set_bit(self, value, bit):
        return value | (bit)

#-----------------------------------------------------------------------------------------------
    def clear_bit(self, value, bit):
        return value & ~(bit)

#-----------------------------------------------------------------------------------------------
    def handle_error(self):
        if self.error_handler_running:
            return
        self.error_handler_running = True
        gstat = self.read_int(reg.GSTAT)
        if gstat == -1:
            self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): No answer from Driver")
        else:
            if gstat & reg.reset:
                self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): The Driver has been reset since the last read access to GSTAT")
            if gstat & reg.drv_err:
                self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): The driver has been shut down due to overtemperature or short circuit detection since the last read access")
            if gstat & reg.uv_cp:
                self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): Undervoltage on the charge pump. The driver is disabled in this case")
        self.ExceptionCounter.Raise()

#-----------------------------------------------------------------------------------------------
    def read_drv_status(self):
        drvstatus =self.read_int(reg.DRVSTATUS)
        self.LogFile.Log("TMC Driver status Motor ID "+str(self.mtr_id))
        if drvstatus & reg.stst:
            self.LogFile.Log("Motor is standing still")
        else:
            self.LogFile.Log("Motor is running")

        if drvstatus & reg.stealth:
            self.LogFile.Log("Motor is running on StealthChop")
        else:
            self.LogFile.Log("Motor is running on SpreadCycle")

        cs_actual = drvstatus & reg.cs_actual
        cs_actual = cs_actual >> 16

        if drvstatus & reg.olb:
            self.LogFile.Log("Open load detected on phase B")

        if drvstatus & reg.ola:
            self.LogFile.Log("Open load detected on phase A")

        if drvstatus & reg.s2vsb:
            self.LogFile.Log("Short on low-side MOSFET detected on phase B. The driver becomes disabled")

        if drvstatus & reg.s2vsa:
            self.LogFile.Log("Short on low-side MOSFET detected on phase A. The driver becomes disabled")

        if drvstatus & reg.s2gb:
            self.LogFile.Log("Short to GND detected on phase B. The driver becomes disabled")

        if drvstatus & reg.s2ga:
            self.LogFile.Log("Short to GND detected on phase A. The driver becomes disabled")

        if drvstatus & reg.ot:
            self.LogFile.Log("Driver Overheating!")

        if drvstatus & reg.otpw:
            self.LogFile.Log("Driver Overheating Prewarning!")

        return drvstatus

#-----------------------------------------------------------------------------------------------
    def read_gconf(self):
        gconf = self.read_int(reg.GCONF)

        if gconf & reg.i_scale_analog:
            self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): Driver is using voltage supplied to VREF as current reference")
        else:
            self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): Driver is using internal reference derived from 5VOUT")
        if gconf & reg.internal_rsense:
            self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): Internal sense resistors. Use current supplied into VREF as reference.")
            self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): VREF pin internally is driven to GND in this mode.")
            self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): This will most likely destroy your driver!!!")
            raise SystemExit
        self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): Operation with external sense resistors")
        if gconf & reg.en_spreadcycle:
            self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): SpreadCycle mode enabled")
        else:
            self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): StealthChop PWM mode enabled")
        if gconf & reg.shaft:
            self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): Inverse motor direction")
        else:
            self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): Normal motor direction")
        if gconf & reg.index_otpw:
            self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): INDEX pin outputs overtemperature prewarning flag")
        else:
            self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): INDEX shows the first microstep position of sequencer")
        if gconf & reg.index_step:
            self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): INDEX output shows step pulses from internal pulse generator")
        else:
            self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): INDEX output as selected by index_otpw")
        if gconf & reg.mstep_reg_select:
            self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): Microstep resolution selected by MSTEP register")
        else:
            self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): Microstep resolution selected by pins MS1, MS2")

        return gconf

#-----------------------------------------------------------------------------------------------
    def read_gstat(self):
        gstat = self.read_int(reg.GSTAT)
        if gstat & reg.reset:
            self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): The Driver has been reset since the last read access to GSTAT")
        if gstat & reg.drv_err:
            self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): The driver has been shut down due to overtemperature or short circuit detection since the last read access")
        if gstat & reg.uv_cp:
            self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): Undervoltage on the charge pump. The driver is disabled in this case")
        return gstat

#-----------------------------------------------------------------------------------------------
    def clear_gstat(self):
        self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): Clearing GSTAT")
        gstat = self.uart.read_int(reg.GSTAT)

        gstat = self.uart.set_bit(gstat, reg.reset)
        gstat = self.uart.set_bit(gstat, reg.drv_err)

        self.write_reg_check(reg.GSTAT, gstat)

#-----------------------------------------------------------------------------------------------
    def read_ioin(self):
        ioin = self.read_int(reg.IOIN)
        return ioin

#-----------------------------------------------------------------------------------------------
    def read_chopconf(self):
        chopconf = self.read_int(reg.CHOPCONF)
        self.LogFile.Log(f"TMCStepper({self.mtr_id}): Native {self.get_microstepping_resolution()} microstep setting")

        if chopconf & reg.intpol:
            self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): Interpolation to 256 µsteps")

        if chopconf & reg.vsense:
            self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): High sensitivity, low sense resistor voltage")
        else:
            self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): Low sensitivity, high sense resistor voltage")
        return chopconf

#-----------------------------------------------------------------------------------------------
    def get_direction_reg(self):
        gconf = self.read_int(reg.GCONF)
        return gconf & reg.shaft

#-----------------------------------------------------------------------------------------------
    def set_direction_reg(self, direction):
        gconf = self.read_int(reg.GCONF)
        if direction:
            self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): Setting inverse motor direction")
            gconf = self.set_bit(gconf, reg.shaft)
        else:
            self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): Setting normal motor direction")
            gconf = self.clear_bit(gconf, reg.shaft)
        self.write_reg_check(reg.GCONF, gconf)
        self._direction = not direction

#-----------------------------------------------------------------------------------------------
    def get_iscale_analog(self):
        gconf = self.read_int(reg.GCONF)
        return gconf & reg.i_scale_analog

#-----------------------------------------------------------------------------------------------
    def set_iscale_analog(self,en):
        gconf = self.read_int(reg.GCONF)
        if en:
            self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): Activated Vref for current scale")
            gconf = self.set_bit(gconf, reg.i_scale_analog)
        else:
            self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): Activated 5V-out for current scale")
            gconf = self.clear_bit(gconf, reg.i_scale_analog)
        self.write_reg_check(reg.GCONF, gconf)

#-----------------------------------------------------------------------------------------------
    def get_vsense(self):
        chopconf = self.read_int(reg.CHOPCONF)
        return chopconf & reg.vsense

#-----------------------------------------------------------------------------------------------
    def set_vsense(self,en):
        chopconf = self.read_int(reg.CHOPCONF)
        if en:
            self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): Activated High sensitivity, low sense resistor voltage")
            chopconf = self.set_bit(chopconf, reg.vsense)
        else:
            self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): Activated Low sensitivity, high sense resistor voltage")
            chopconf = self.clear_bit(chopconf, reg.vsense)
        self.write_reg_check(reg.CHOPCONF, chopconf)

#-----------------------------------------------------------------------------------------------
    def get_internal_rsense(self):
        gconf = self.read_int(reg.GCONF)
        return gconf & reg.internal_rsense

#-----------------------------------------------------------------------------------------------
    def set_internal_rsense(self,en):
        gconf = self.read_int(reg.GCONF)
        if en:
            self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): Activated internal sense resistors.")
            self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): VREF pin internally is driven to GND in this mode.")
            self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): This will most likely destroy your driver!!!")
            raise SystemExit
        self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): Activated operation with external sense resistors")
        gconf = self.clear_bit(gconf, reg.internal_rsense)
        self.write_reg_check(reg.GCONF, gconf)

#-----------------------------------------------------------------------------------------------
    def set_irun_ihold(self, ihold, irun, ihold_delay):
        ihold_irun = 0

        ihold_irun = ihold_irun | ihold << 0
        ihold_irun = ihold_irun | irun << 8
        ihold_irun = ihold_irun | ihold_delay << 16
        self.write_reg_check(reg.IHOLD_IRUN, ihold_irun)

#-----------------------------------------------------------------------------------------------
    def set_pdn_disable(self,pdn_disable):
        gconf = self.read_int(reg.GCONF)
        if pdn_disable:
            self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): Enabled PDN_UART")
            gconf = self.set_bit(gconf, reg.pdn_disable)
        else:
            self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): Disabled PDN_UART")
            gconf = self.clear_bit(gconf, reg.pdn_disable)
        self.write_reg_check(reg.GCONF, gconf)

#-----------------------------------------------------------------------------------------------
    def set_current(self, run_current, hold_current_multiplier = 0.5,
                    hold_current_delay = 10, pdn_disable = True):
        cs_irun = 0
        rsense = 0.11
        vfs = 0

        self.set_iscale_analog(False)

        vfs = 0.325
        cs_irun = 32.0*1.41421*run_current/1000.0*(rsense+0.02)/vfs - 1

        # If Current Scale is too low, turn on high sensitivity VSsense and calculate again
        if cs_irun < 16:
            self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): CS too low; switching to VSense True")
            vfs = 0.180
            cs_irun = 32.0*1.41421*run_current/1000.0*(rsense+0.02)/vfs - 1
            self.set_vsense(True)
        else: # If CS >= 16, turn off high_senser
            self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): CS in range; using VSense False")
            self.set_vsense(False)

        cs_irun = min(cs_irun, 31)
        cs_irun = max(cs_irun, 0)

        CS_IHold = hold_current_multiplier * cs_irun

        cs_irun = round(cs_irun)
        CS_IHold = round(CS_IHold)
        hold_current_delay = round(hold_current_delay)

        run_current_actual = (cs_irun+1)/32.0 * (vfs)/(rsense+0.02) / 1.41421 * 1000
        self.LogFile.Log(f"TMCStepper({self.mtr_id}): Actual current: {round(run_current_actual)} mA")

        self.set_irun_ihold(CS_IHold, cs_irun, hold_current_delay)
        self.set_pdn_disable(pdn_disable)

#-----------------------------------------------------------------------------------------------
    def get_spreadcycle(self):
        gconf = self.read_int(reg.GCONF)
        return gconf & reg.en_spreadcycle

#-----------------------------------------------------------------------------------------------
    def set_spreadcycle(self,en_spread):
        gconf = self.read_int(reg.GCONF)
        if en_spread:
            self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): Activated Spreadcycle")
            gconf = self.set_bit(gconf, reg.en_spreadcycle)
        else:
            self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): Activated Stealthchop")
            gconf = self.clear_bit(gconf, reg.en_spreadcycle)
        self.write_reg_check(reg.GCONF, gconf)

#-----------------------------------------------------------------------------------------------
    def get_interpolation(self):
        chopconf = self.read_int(reg.CHOPCONF)
        return bool(chopconf & reg.intpol)

#-----------------------------------------------------------------------------------------------
    def set_interpolation(self, en):
        chopconf = self.read_int(reg.CHOPCONF)

        if en:
            chopconf = self.set_bit(chopconf, reg.intpol)
        else:
            chopconf = self.clear_bit(chopconf, reg.intpol)

        self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): Writing microstep interpolation setting: "+str(en))
        self.write_reg_check(reg.CHOPCONF, chopconf)

#-----------------------------------------------------------------------------------------------
    def get_toff(self):
        chopconf = self.read_int(reg.CHOPCONF)

        toff = chopconf & (reg.toff0 | reg.toff1 | reg.toff2 | reg.toff3)
        toff = toff >> 0
        return toff

#-----------------------------------------------------------------------------------------------
    def set_toff(self, toff):
        # Ensure toff is a four-bit value by zeroing out the top bits
        toff = toff & 0x0F

        # Read the current value of the CHOPCONF register
        chopconf = self.read_int(reg.CHOPCONF)

        # Zero out the lower four bits of the CHOPCONF register
        chopconf = chopconf & 0xFFFFFFF0

        # Set the lower four bits of CHOPCONF to the toff value
        chopconf = chopconf | toff

        # Write the new value back to the CHOPCONF register
        self.write_reg_check(reg.CHOPCONF, chopconf)

        # Log the action
        self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): Writing toff setting: "+str(toff))

#-----------------------------------------------------------------------------------------------
    def read_microstepping_resolution(self):
        chopconf = self.read_int(reg.CHOPCONF)

        msresdezimal = chopconf & (reg.msres0 | reg.msres1 | reg.msres2 | reg.msres3)
        msresdezimal = msresdezimal >> 24
        msresdezimal = 8 - msresdezimal

        self._msres = int(math.pow(2, msresdezimal))
        self._steps_per_rev = self._fullsteps_per_rev * self._msres
        
        return self._msres

#-----------------------------------------------------------------------------------------------
    def get_microstepping_resolution(self):
        return self._msres

#-----------------------------------------------------------------------------------------------
    def set_microstepping_resolution(self, msres):
        chopconf = self.read_int(reg.CHOPCONF)
        #setting all bits to zero
        chopconf = chopconf & (~reg.msres0 & ~reg.msres1 &
                                ~reg.msres2 & ~reg.msres3)
        msresdezimal = int(math.log(msres, 2))
        msresdezimal = 8 - msresdezimal
        chopconf = chopconf | msresdezimal <<24

        self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): Writing "+str(msres)+" microstep setting")
        self.write_reg_check(reg.CHOPCONF, chopconf)

        self._msres = msres
        self._steps_per_rev = self._fullsteps_per_rev * self._msres
        self.set_mstep_resolution_reg_select(True)

        return True

#-----------------------------------------------------------------------------------------------
    def set_mstep_resolution_reg_select(self, en):
        gconf = self.read_int(reg.GCONF)

        if en is True:
            gconf = self.set_bit(gconf, reg.mstep_reg_select)
        else:
            gconf = self.clear_bit(gconf, reg.mstep_reg_select)

        self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): Writing MStep Reg Select: "+str(en))
        self.write_reg_check(reg.GCONF, gconf)

#-----------------------------------------------------------------------------------------------
    def get_interface_transmission_counter(self):
        ifcnt = self.read_int(reg.IFCNT)
        return ifcnt

#-----------------------------------------------------------------------------------------------
    def get_tstep(self):
        tstep = self.read_int(reg.TSTEP)
        return tstep

#-----------------------------------------------------------------------------------------------
    def set_vactual(self, vactual):
        self.write_reg_check(reg.VACTUAL, vactual)

#-----------------------------------------------------------------------------------------------
    def get_stallguard_result(self):
        sg_result = self.read_int(reg.SG_RESULT)
        return sg_result

#-----------------------------------------------------------------------------------------------
    def set_stallguard_threshold(self, threshold):
        self.write_reg_check(reg.SGTHRS, threshold)

#-----------------------------------------------------------------------------------------------
    def set_coolstep_threshold(self, threshold):
        self.write_reg_check(reg.TCOOLTHRS, threshold)

#-----------------------------------------------------------------------------------------------
    def get_microstep_counter(self):
        mscnt = self.read_int(reg.MSCNT)
        return mscnt

#-----------------------------------------------------------------------------------------------
    def get_microstep_counter_in_steps(self, offset=0):
        step = (self.get_microstep_counter()-64)*(self._msres*4)/1024
        step = (4*self._msres)-step-1
        step = round(step)
        return step+offset

#-----------------------------------------------------------------------------------------------
    def set_direction_pin(self, direction):
        if self._pin_dir != -1:
            self._direction = direction
            self._pin_dir.SetValue(direction)
        else:
            self.LogFile.Log("TMCStepper("+str(self.mtr_id)+"): Direction pin not defined.")
            
#-----------------------------------------------------------------------------------------------
    def set_direction_pin_or_reg(self, direction):
        if self._pin_dir != -1:
            self.set_direction_pin(direction)
        else:
            self.set_direction_reg(not direction) #no clue, why this has to be inverted

#-----------------------------------------------------------------------------------------------
    def set_vactual_dur(self, vactual, duration=0, acceleration=0,
                        show_stallguard_result=False, show_tstep=False):
 
        self._stop = self.NO
        current_vactual = 0
        sleeptime = 0.05
        time_to_stop = 0
        if vactual<0:
            acceleration = -acceleration

        if acceleration == 0:
            self.set_vactual(int(round(vactual)))

        if duration == 0:
            return -1

        self._starttime = time.time()
        current_time = time.time()
        while current_time < self._starttime+duration:
            if self._stop == self.HARDSTOP:
                break
            if acceleration != 0:
                time_to_stop = self._starttime+duration-abs(current_vactual/acceleration)
                if self._stop == self.SOFTSTOP:
                    time_to_stop = current_time-1
            if acceleration != 0 and current_time > time_to_stop:
                current_vactual -= acceleration*sleeptime
                self.set_vactual(int(round(current_vactual)))
                time.sleep(sleeptime)
            elif acceleration != 0 and abs(current_vactual)<abs(vactual):
                current_vactual += acceleration*sleeptime
                self.set_vactual(int(round(current_vactual)))
                time.sleep(sleeptime)
            if show_stallguard_result:
                print(f"StallGuard result: {self.get_stallguard_result()}")
                time.sleep(0.1)
            if show_tstep:
                print(f"TStep result: {self.get_tstep()}")
                time.sleep(0.1)
            current_time = time.time()
        self.set_vactual(0)
        return self._stop

#-----------------------------------------------------------------------------------------------
    def set_movement_abs_rel(self, movement_abs_rel):
        self._movement_abs_rel = movement_abs_rel

#-----------------------------------------------------------------------------------------------
    def get_current_position(self):
        return self._current_pos

#-----------------------------------------------------------------------------------------------
    def set_current_position(self, new_pos):
        self._current_pos = new_pos
        
#-----------------------------------------------------------------------------------------------
    def set_speed(self, speed):
        # Sets the motor speed in steps per second

        if speed == self._speed:
            return
        speed = self.constrain(speed, -self._max_speed, self._max_speed)
        if speed == 0.0:
            self._step_interval = 0
        else:
            self._step_interval = abs(1000000.0 / speed)
            if speed > 0:
                self.set_direction_pin_or_reg(1)
            else:
                self.set_direction_pin_or_reg(0)
        self._speed = speed

#-----------------------------------------------------------------------------------------------
    def set_speed_fullstep(self, speed):
        self.set_speed(speed*self.get_microstepping_resolution())


#-----------------------------------------------------------------------------------------------
    def set_max_speed(self, speed):
        # Sets the maximum motor speed in microsteps per second

        if speed < 0.0:
            speed = -speed
        if self._max_speed != speed:
            self._max_speed = speed
            if speed == 0.0:
                self._cmin = 0.0
            else:
                self._cmin = 1000000.0 / speed
            # Recompute _n from current speed and adjust speed if accelerating or cruising
            if self._n > 0:
                self._n = (self._speed * self._speed) / (2.0 * self._acceleration) # Equation 16
                self.compute_new_speed()

#-----------------------------------------------------------------------------------------------
    def set_max_speed_fullstep(self, speed):
        self.set_max_speed(speed*self.get_microstepping_resolution())

#-----------------------------------------------------------------------------------------------
    def get_max_speed(self):
        return self._max_speed

#-----------------------------------------------------------------------------------------------
    def set_acceleration(self, acceleration):
        # Sets the motor acceleration/deceleration in microsteps per sec per sec
        if acceleration == 0.0:
            return
        acceleration = abs(acceleration)
        if self._acceleration != acceleration:
            self._n = self._n * (self._acceleration / acceleration)
            self._c0 = 0.676 * math.sqrt(2.0 / acceleration) * 1000000.0 # Equation 15
            self._acceleration = acceleration
            self.compute_new_speed()

#-----------------------------------------------------------------------------------------------
    def set_acceleration_fullstep(self, acceleration):
        self.set_acceleration(acceleration*self.get_microstepping_resolution())

#-----------------------------------------------------------------------------------------------
    def get_acceleration(self):
        return self._acceleration

#-----------------------------------------------------------------------------------------------
    def stop(self, stop_mode = StopMode.HARDSTOP):
        self._stop = stop_mode

#-----------------------------------------------------------------------------------------------
    def get_movement_phase(self):
        return self._movement_phase

#-----------------------------------------------------------------------------------------------
    def run_to_position_steps(self, steps, callback=None):
        self._target_pos = self._current_pos + steps       
        self._stop = StopMode.NO
        self._step_interval = 0
        self._speed = 0.0
        self._n = 0
        self.compute_new_speed()
        while self.run(): #returns false, when target position is reached
            if (callback != None): callback()
            if self._stop == StopMode.HARDSTOP:
                break

        self._movement_phase = MovementPhase.STANDSTILL
        return self._stop

#-----------------------------------------------------------------------------------------------
    def run(self):
        if self.run_speed(): #returns true, when a step is made
            self.compute_new_speed()
        return self._speed != 0.0 and self.distance_to_go() != 0

#-----------------------------------------------------------------------------------------------
    def distance_to_go(self):
        return self._target_pos - self._current_pos

#-----------------------------------------------------------------------------------------------
    def compute_new_speed(self):
        # Generate stepper-motor speed profiles in real time" by David Austin
        # https://www.embedded.com/generate-stepper-motor-speed-profiles-in-real-time/

        distance_to = self.distance_to_go() # +ve is clockwise from current location
        steps_to_stop = (self._speed * self._speed) / (2.0 * self._acceleration) # Equation 16
        if ((distance_to == 0 and steps_to_stop <= 2) or
        (self._stop == StopMode.SOFTSTOP and steps_to_stop <= 1)):
            # We are at the target and its time to stop
            self._step_interval = 0
            self._speed = 0.0
            self._n = 0
            self._movement_phase = MovementPhase.STANDSTILL
            return

        if distance_to > 0:
            # We are anticlockwise from the target
            # Need to go clockwise from here, maybe decelerate now
            if self._n > 0:
                # Currently accelerating, need to decel now? Or maybe going the wrong way?
                if ((steps_to_stop >= distance_to) or self._direction == Direction.CCW or
                    self._stop == StopMode.SOFTSTOP):
                    self._n = -steps_to_stop # Start deceleration
                    self._movement_phase = MovementPhase.DECELERATING
            elif self._n < 0:
                # Currently decelerating, need to accel again?
                if (steps_to_stop < distance_to) and self._direction == Direction.CW:
                    self._n = -self._n # Start acceleration
                    self._movement_phase = MovementPhase.ACCELERATING
        elif distance_to < 0:
            # We are clockwise from the target
            # Need to go anticlockwise from here, maybe decelerate
            if self._n > 0:
                # Currently accelerating, need to decel now? Or maybe going the wrong way?
                if (((steps_to_stop >= -distance_to) or self._direction == Direction.CW or
                    self._stop == StopMode.SOFTSTOP)):
                    self._n = -steps_to_stop # Start deceleration
                    self._movement_phase = MovementPhase.DECELERATING
            elif self._n < 0:
                # Currently decelerating, need to accel again?
                if (steps_to_stop < -distance_to) and self._direction == Direction.CCW:
                    self._n = -self._n # Start acceleration
                    self._movement_phase = MovementPhase.ACCELERATING
        # Need to accelerate or decelerate
        if self._n == 0:
            # First step from stopped
            self._cn = self._c0
            self._pin_step.SetValue(False)
            if distance_to > 0:
                self.set_direction_pin_or_reg(1)
            else:
                self.set_direction_pin_or_reg(0)
            self._movement_phase = MovementPhase.ACCELERATING
        else:
            # Subsequent step. Works for accel (n is +_ve) and decel (n is -ve).
            self._cn = self._cn - ((2.0 * self._cn) / ((4.0 * self._n) + 1)) # Equation 13
            self._cn = max(self._cn, self._cmin)
            if self._cn == self._cmin:
                self._movement_phase = MovementPhase.MAXSPEED
        self._n += 1
        self._step_interval = self._cn
        self._speed = 1000000.0 / self._cn
        if self._direction == 0:
            self._speed = -self._speed

#-----------------------------------------------------------------------------------------------
    def run_speed(self):
        # Don't do anything unless we actually have a step interval
        if not self._step_interval:
            return False
        curtime = time.monotonic_ns()/1000
        if curtime - self._last_step_time >= self._step_interval:

            if self._direction == 1: # Clockwise
                self._current_pos += 1
            else: # Anticlockwise
                self._current_pos -= 1
            self.make_a_step()
            self._last_step_time = curtime # Caution: does not account for costs in step()

            return True
        return False

#-----------------------------------------------------------------------------------------------
    def make_a_step(self):
        self._pin_step.SetValue(True)
        time.sleep(1/1000/1000)
        self._pin_step.SetValue(False)
        time.sleep(1/1000/1000)

#-----------------------------------------------------------------------------------------------
    def rps_to_vactual(self, rps, steps_per_rev, fclk = 12000000):
        # rps (float): revolutions per second
        # steps_per_rev (int): steps per revolution
        # fclk (int): clock speed of the tmc (Default value = 12000000)

        return int(round(rps / (fclk / 16777216) * steps_per_rev))


#-----------------------------------------------------------------------------------------------
    def vactual_to_rps(self, vactual, steps_per_rev, fclk = 12000000):
        # vactual (int): value for VACTUAL
        # steps_per_rev (int): steps per revolution
        # fclk (int): clock speed of the tmc (Default value = 12000000)

        return vactual * (fclk / 16777216) / steps_per_rev

#-----------------------------------------------------------------------------------------------
    def rps_to_steps(self, rps, steps_per_rev):
        return rps * steps_per_rev

#-----------------------------------------------------------------------------------------------
    def steps_to_rps(self, steps, steps_per_rev):
        return steps / steps_per_rev

#-----------------------------------------------------------------------------------------------
    def rps_to_tstep(self, rps, steps_per_rev, msres):
        return int(round(12000000 / (rps_to_steps(rps, steps_per_rev) * 256 / msres)))

#-----------------------------------------------------------------------------------------------
    def steps_to_tstep(self, steps, msres):
        return int(round(12000000 / (steps * 256 / msres)))

#-----------------------------------------------------------------------------------------------
    def constrain(self, val, min_val, max_val):
        if val < min_val:
            return min_val
        if val > max_val:
            return max_val
        return val
