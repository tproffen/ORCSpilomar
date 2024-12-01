import time
import board
import busio
import digitalio
import math
import struct

from . import tmc2209_reg as reg

class FakeEnumType(type):
    def __init__(self, name, bases, namespace):
        # print("FakeEnumType.__init__", self, name, bases)
        #self.enumclass = type(name, (FakeEnum,), {})
        super().__init__(name, bases, namespace)
    def __setattr__(self, name, value):
        # print("FakeEnumType.__setattr__")
        if not name.startswith('_'):
            raise AttributeError("Cannot reassign enum members")
        super().__setattr__(name, value)
    def __getattribute__(self, __name: str):
        # print("FakeEnumType.__getattribute__")
        if __name.startswith('_'):
            return super().__getattribute__(__name)
        return self(self.__dict__[__name])
    def __contains__(self, item):
        return item in self.__iter__()
    def __iter__(self):
        for key, value in self.__dict__.items():
            if key.startswith('_'):
                continue
            try:
                yield self(value)
            except (TypeError, ValueError):
                pass    # Not a valid enum value


class Enum(FakeEnumType):

    def __init__(self, value):
        if isinstance(value, self.__class__):
            value = value.value
        else:
            self.value = value
        # print("FakeEnum.__init__")

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return self.value == other.value
        return NotImplemented

    def __ne__(self, other):
        result = self.__eq__(other)
        if result is NotImplemented:
            return result
        return not result

    def __repr__(self):
        return f"{self.__class__.__name__} {self.value}"

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

def sleep_ns(ns):
    start=time.monotonic_ns()
    while (time.monotonic_ns()-start < ns):
        pass

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

    NO = 0
    SOFTSTOP = 1
    HARDSTOP = 2


    _direction = True

    _stop = StopMode.NO
    _starttime = 0
    _sg_callback = None

    _msres = -1
    _steps_per_rev = 0
    _fullsteps_per_rev = 200

    _current_pos = 0                 # current position of stepper in steps
    _target_pos = 0                  # the target position in steps
    _speed = 0.0                    # the current speed in steps per second
    _max_speed = 1.0                 # the maximum speed in steps per second
    _max_speed_homing = 200           # the maximum speed in steps per second for homing
    _acceleration = 1.0             # the acceleration in steps per second per second
    _acceleration_homing = 10000     # the acceleration in steps per second per second for homing
    _sqrt_twoa = 1.0                # Precomputed sqrt(2*_acceleration)
    _step_interval = 0               # the current interval between two steps
    _min_pulse_width = 1              # minimum allowed pulse with in microseconds
    _last_step_time = 0               # The last step time in microseconds
    _n = 0                          # step counter
    _c0 = 0                         # Initial step size in microseconds
    _cn = 0                         # Last step size in microseconds
    _cmin = 0                       # Min step size in microseconds based on maxSpeed
    _sg_threshold = 100             # threshold for stallguard
    _movement_abs_rel = MovementAbsRel.ABSOLUTE
    _movement_phase = MovementPhase.STANDSTILL

    _pin_dir = -1
#    _pin_step = digitalio.DigitalInOut(board.GP29)
#    _pin_step.direction = digitalio.Direction.OUTPUT

    def __init__(self, uart, mtr_id = 0, communication_pause = 0.004):
        self.mtr_id = mtr_id
        self.communication_pause = communication_pause
        self.uart = uart


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

    def read_reg(self, register):
        self.uart.reset_input_buffer()

        self.r_frame[1] = self.mtr_id
        self.r_frame[2] = register
        self.r_frame[3] = self.compute_crc8_atm(self.r_frame[:-1])

        rtn = self.uart.write(bytearray(self.r_frame))
        if rtn != len(self.r_frame):
            print("Err in read")
            return False

        time.sleep(self.communication_pause)
        rtn = self.uart.read(12)
        #print(f"received {len(rtn)} bytes; {len(rtn*8)} bits")
        time.sleep(self.communication_pause)

        return rtn

    def read_int(self, register, tries=10):
        while True:
            tries -= 1
            rtn = self.read_reg(register)
            rtn_data = rtn[7:11]
            not_zero_count = len([elem for elem in rtn if elem != 0])
            if(len(rtn)<12 or not_zero_count == 0):
                print(f"""UART Communication Error: {len(rtn_data)} data bytes | {len(rtn)} total bytes""")
            elif rtn[11] != self.compute_crc8_atm(rtn[4:11]):
                print("UART Communication Error: CRC MISMATCH")
            else:
                break

            if tries<=0:
                print("after 10 tries not valid answer")
                print(f"snd:\t{bytes(self.r_frame)}")
                print(f"rtn:\t{rtn}")
                self.handle_error()
                return -1

        val = struct.unpack(">i",rtn_data)[0]
        return val

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
            print("Err in write")
            return False
        time.sleep(self.communication_pause)
        return True



    def write_reg_check(self, register, val, tries=10):
        ifcnt1 = self.read_int(reg.IFCNT)

        if ifcnt1 == 255:
            ifcnt1 = -1

        while True:
            self.write_reg(register, val)
            tries -= 1
            ifcnt2 = self.read_int(reg.IFCNT)
            if ifcnt1 >= ifcnt2:
                print("writing not successful!")
                print(f"ifcnt: {ifcnt1}, {ifcnt2}")
            else:
                return True
            if tries<=0:
                print("after 10 tries no valid write access")
                self.handle_error()
                return -1

    def set_bit(self, value, bit):
        return value | (bit)

    def clear_bit(self, value, bit):
        return value & ~(bit)

    def handle_error(self):
        if self.error_handler_running:
            return
        self.error_handler_running = True
        gstat = self.read_int(reg.GSTAT)
        print("GSTAT Error check:")
        if gstat == -1:
            print("No answer from Driver")
        elif gstat == 0:
            print("Everything looks fine in GSTAT")
        else:
            if gstat & reg.reset:
                print("The Driver has been reset since the last read access to GSTAT")
            if gstat & reg.drv_err:
                print("""The driver has been shut down due to overtemperature or short
                      circuit detection since the last read access""")
            if gstat & reg.uv_cp:
                print("""Undervoltage on the charge pump.
                      The driver is disabled in this case""")
        print("EXITING!")

    def read_drv_status(self):
        drvstatus =self.read_int(reg.DRVSTATUS)
        if drvstatus & reg.stst:
            print("Motor is standing still")
        else:
            print("Motor is running")

        if drvstatus & reg.stealth:
            print("Motor is running on StealthChop")
        else:
            print("Motor is running on SpreadCycle")

        cs_actual = drvstatus & reg.cs_actual
        cs_actual = cs_actual >> 16
        print(f"CS actual: {cs_actual}")

        if drvstatus & reg.olb:
            print("Open load detected on phase B")

        if drvstatus & reg.ola:
            print("Open load detected on phase A")

        if drvstatus & reg.s2vsb:
            print("""Short on low-side MOSFET detected on phase B.
                        The driver becomes disabled""")

        if drvstatus & reg.s2vsa:
            print("""Short on low-side MOSFET detected on phase A.
                        The driver becomes disabled""")

        if drvstatus & reg.s2gb:
            print("""Short to GND detected on phase B.
                                The driver becomes disabled.""")

        if drvstatus & reg.s2ga:
            print("""Short to GND detected on phase A.
                                The driver becomes disabled.""")

        if drvstatus & reg.ot:
            print("Driver Overheating!")

        if drvstatus & reg.otpw:
            print("Driver Overheating Prewarning!")

        print("---")
        return drvstatus


    def read_gconf(self):
        gconf = self.read_int(reg.GCONF)

        if gconf & reg.i_scale_analog:
            print("Driver is using voltage supplied to VREF as current reference")
        else:
            print("Driver is using internal reference derived from 5VOUT")
        if gconf & reg.internal_rsense:
            print("""Internal sense resistors.
                                Use current supplied into VREF as reference.""")
            print("VREF pin internally is driven to GND in this mode.")
            print("This will most likely destroy your driver!!!")
            raise SystemExit
        print("Operation with external sense resistors")
        if gconf & reg.en_spreadcycle:
            print("SpreadCycle mode enabled")
        else:
            print("StealthChop PWM mode enabled")
        if gconf & reg.shaft:
            print("Inverse motor direction")
        else:
            print("Normal motor direction")
        if gconf & reg.index_otpw:
            print("INDEX pin outputs overtemperature prewarning flag")
        else:
            print("INDEX shows the first microstep position of sequencer")
        if gconf & reg.index_step:
            print("INDEX output shows step pulses from internal pulse generator")
        else:
            print("INDEX output as selected by index_otpw")
        if gconf & reg.mstep_reg_select:
            print("Microstep resolution selected by MSTEP register")
        else:
            print("Microstep resolution selected by pins MS1, MS2")

        print("---")
        return gconf


    def read_gstat(self):
        gstat = self.read_int(reg.GSTAT)
        if gstat & reg.reset:
            print("The Driver has been reset since the last read access to GSTAT")
        if gstat & reg.drv_err:
            print("""The driver has been shut down due to overtemperature or
                        short circuit detection since the last read access""")
        if gstat & reg.uv_cp:
            print("""Undervoltage on the charge pump.
                        The driver is disabled in this case""")
        return gstat

    def clear_gstat(self):
        print("clearing GSTAT")
        gstat = self.uart.read_int(reg.GSTAT)

        gstat = self.uart.set_bit(gstat, reg.reset)
        gstat = self.uart.set_bit(gstat, reg.drv_err)

        self.write_reg_check(reg.GSTAT, gstat)

    def read_ioin(self):
        ioin = self.read_int(reg.IOIN)
        print(bin(ioin))
        if ioin & reg.io_spread:
            print("spread is high")
        else:
            print("spread is low")

        if ioin & reg.io_dir:
            print("dir is high")
        else:
            print("dir is low")

        if ioin & reg.io_step:
            print("step is high")
        else:
            print("step is low")

        if ioin & reg.io_enn:
            print("en is high")
        else:
            print("en is low")

        print("---")
        return ioin



    def read_chopconf(self):
        print("---")
        print("CHOPPER CONTROL")
        chopconf = self.read_int(reg.CHOPCONF)
        print(bin(chopconf))

        print(f"native {self.get_microstepping_resolution()} microstep setting")

        if chopconf & reg.intpol:
            print("interpolation to 256 µsteps")

        if chopconf & reg.vsense:
            print("1: High sensitivity, low sense resistor voltage")
        else:
            print("0: Low sensitivity, high sense resistor voltage")

        print("---")
        return chopconf



    def get_direction_reg(self):
        gconf = self.read_int(reg.GCONF)
        return gconf & reg.shaft



    def set_direction_reg(self, direction):
        gconf = self.read_int(reg.GCONF)
        if direction:
            print("write inverse motor direction")
            gconf = self.set_bit(gconf, reg.shaft)
        else:
            print("write normal motor direction")
            gconf = self.clear_bit(gconf, reg.shaft)
        self.write_reg_check(reg.GCONF, gconf)
        self._direction = not direction



    def get_iscale_analog(self):
        gconf = self.read_int(reg.GCONF)
        return gconf & reg.i_scale_analog



    def set_iscale_analog(self,en):
        gconf = self.read_int(reg.GCONF)
        if en:
            print("activated Vref for current scale")
            gconf = self.set_bit(gconf, reg.i_scale_analog)
        else:
            print("activated 5V-out for current scale")
            gconf = self.clear_bit(gconf, reg.i_scale_analog)
        self.write_reg_check(reg.GCONF, gconf)



    def get_vsense(self):
        chopconf = self.read_int(reg.CHOPCONF)
        return chopconf & reg.vsense



    def set_vsense(self,en):
        chopconf = self.read_int(reg.CHOPCONF)
        if en:
            print("activated High sensitivity, low sense resistor voltage")
            chopconf = self.set_bit(chopconf, reg.vsense)
        else:
            print("activated Low sensitivity, high sense resistor voltage")
            chopconf = self.clear_bit(chopconf, reg.vsense)
        self.write_reg_check(reg.CHOPCONF, chopconf)



    def get_internal_rsense(self):
        gconf = self.read_int(reg.GCONF)
        return gconf & reg.internal_rsense



    def set_internal_rsense(self,en):
        gconf = self.read_int(reg.GCONF)
        if en:
            print("activated internal sense resistors.")
            print("VREF pin internally is driven to GND in this mode.")
            print("This will most likely destroy your driver!!!")
            raise SystemExit
            # gconf = self.set_bit(gconf, reg.internal_rsense)
        print("activated operation with external sense resistors")
        gconf = self.clear_bit(gconf, reg.internal_rsense)
        self.write_reg_check(reg.GCONF, gconf)



    def set_irun_ihold(self, ihold, irun, ihold_delay):
        ihold_irun = 0

        ihold_irun = ihold_irun | ihold << 0
        ihold_irun = ihold_irun | irun << 8
        ihold_irun = ihold_irun | ihold_delay << 16
        print(f"ihold_irun: {bin(ihold_irun)}")

        print("writing ihold_irun")
        self.write_reg_check(reg.IHOLD_IRUN, ihold_irun)



    def set_pdn_disable(self,pdn_disable):
        gconf = self.read_int(reg.GCONF)
        if pdn_disable:
            print("enabled PDN_UART")
            gconf = self.set_bit(gconf, reg.pdn_disable)
        else:
            print("disabled PDN_UART")
            gconf = self.clear_bit(gconf, reg.pdn_disable)
        self.write_reg_check(reg.GCONF, gconf)



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
            print("CS too low; switching to VSense True")
            vfs = 0.180
            cs_irun = 32.0*1.41421*run_current/1000.0*(rsense+0.02)/vfs - 1
            self.set_vsense(True)
        else: # If CS >= 16, turn off high_senser
            print("CS in range; using VSense False")
            self.set_vsense(False)

        cs_irun = min(cs_irun, 31)
        cs_irun = max(cs_irun, 0)

        CS_IHold = hold_current_multiplier * cs_irun

        cs_irun = round(cs_irun)
        CS_IHold = round(CS_IHold)
        hold_current_delay = round(hold_current_delay)

        print(f"cs_irun: {cs_irun}")
        print(f"CS_IHold: {CS_IHold}")
        print(f"Delay: {hold_current_delay}")

        # return (float)(CS+1)/32.0 * (vsense() ? 0.180 : 0.325)/(rsense+0.02) / 1.41421 * 1000;
        run_current_actual = (cs_irun+1)/32.0 * (vfs)/(rsense+0.02) / 1.41421 * 1000
        print(f"actual current: {round(run_current_actual)} mA")

        self.set_irun_ihold(CS_IHold, cs_irun, hold_current_delay)

        self.set_pdn_disable(pdn_disable)



    def get_spreadcycle(self):
        gconf = self.read_int(reg.GCONF)
        return gconf & reg.en_spreadcycle



    def set_spreadcycle(self,en_spread):
        gconf = self.read_int(reg.GCONF)
        if en_spread:
            print("activated Spreadcycle")
            gconf = self.set_bit(gconf, reg.en_spreadcycle)
        else:
            print("activated Stealthchop")
            gconf = self.clear_bit(gconf, reg.en_spreadcycle)
        self.write_reg_check(reg.GCONF, gconf)



    def get_interpolation(self):
        chopconf = self.read_int(reg.CHOPCONF)
        return bool(chopconf & reg.intpol)



    def set_interpolation(self, en):
        chopconf = self.read_int(reg.CHOPCONF)

        if en:
            chopconf = self.set_bit(chopconf, reg.intpol)
        else:
            chopconf = self.clear_bit(chopconf, reg.intpol)

        print(f"writing microstep interpolation setting: {str(en)}")
        self.write_reg_check(reg.CHOPCONF, chopconf)



    def get_toff(self):
        chopconf = self.read_int(reg.CHOPCONF)

        toff = chopconf & (reg.toff0 | reg.toff1 |
                            reg.toff2 | reg.toff3)

        toff = toff >> 0

        return toff



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
        print(f"writing toff setting: {str(toff)}")



    def read_microstepping_resolution(self):
        chopconf = self.read_int(reg.CHOPCONF)

        msresdezimal = chopconf & (reg.msres0 | reg.msres1 |
                                    reg.msres2 | reg.msres3)

        msresdezimal = msresdezimal >> 24
        msresdezimal = 8 - msresdezimal

        self._msres = int(math.pow(2, msresdezimal))
        self._steps_per_rev = self._fullsteps_per_rev * self._msres

        return self._msres



    def get_microstepping_resolution(self):
        return self._msres



    def set_microstepping_resolution(self, msres):
        chopconf = self.read_int(reg.CHOPCONF)
        #setting all bits to zero
        chopconf = chopconf & (~reg.msres0 & ~reg.msres1 &
                                ~reg.msres2 & ~reg.msres3)
        msresdezimal = int(math.log(msres, 2))
        msresdezimal = 8 - msresdezimal
        chopconf = chopconf | msresdezimal <<24

        print(f"writing {msres} microstep setting")
        self.write_reg_check(reg.CHOPCONF, chopconf)

        self._msres = msres
        self._steps_per_rev = self._fullsteps_per_rev * self._msres

        self.set_mstep_resolution_reg_select(True)

        return True



    def set_mstep_resolution_reg_select(self, en):
        gconf = self.read_int(reg.GCONF)

        if en is True:
            gconf = self.set_bit(gconf, reg.mstep_reg_select)
        else:
            gconf = self.clear_bit(gconf, reg.mstep_reg_select)

        print(f"writing MStep Reg Select: {en}")
        self.write_reg_check(reg.GCONF, gconf)



    def get_interface_transmission_counter(self):
        ifcnt = self.read_int(reg.IFCNT)
        print(f"Interface Transmission Counter: {ifcnt}")
        return ifcnt



    def get_tstep(self):
        tstep = self.read_int(reg.TSTEP)
        return tstep



    def set_vactual(self, vactual):
        self.write_reg_check(reg.VACTUAL, vactual)



    def get_stallguard_result(self):
        sg_result = self.read_int(reg.SG_RESULT)
        return sg_result



    def set_stallguard_threshold(self, threshold):
        print(f"sgthrs {bin(threshold)}")

        print("writing sgthrs")
        self.write_reg_check(reg.SGTHRS, threshold)



    def set_coolstep_threshold(self, threshold):
        print(f"tcoolthrs {bin(threshold)}")

        print("writing tcoolthrs")
        self.write_reg_check(reg.TCOOLTHRS, threshold)



    def get_microstep_counter(self):
        mscnt = self.read_int(reg.MSCNT)
        return mscnt



    def get_microstep_counter_in_steps(self, offset=0):
        step = (self.get_microstep_counter()-64)*(self._msres*4)/1024
        step = (4*self._msres)-step-1
        step = round(step)
        return step+offset

    def set_direction_pin_or_reg(self, direction):
        """sets the motor shaft direction to the given value: 0 = CCW; 1 = CW
        will use the reg, if pin==-1, otherwise use the pin

        Args:
            direction (bool): motor shaft direction: False = CCW; True = CW
        """
        if self._pin_dir != -1:
            pass
            #self.set_direction_pin(direction)
        else:
            self.set_direction_reg(not direction) #no clue, why this has to be inverted


    def set_vactual_dur(self, vactual, duration=0, acceleration=0,
                             show_stallguard_result=False, show_tstep=False):
        """sets the register bit "VACTUAL" to to a given value
        VACTUAL allows moving the motor by UART control.
        It gives the motor velocity in +-(2^23)-1 [μsteps / t]
        0: Normal operation. Driver reacts to STEP input

        Args:
            vactual (int): value for VACTUAL
            duration (int): after this vactual will be set to 0 (Default value = 0)
            acceleration (int): use this for a velocity ramp (Default value = 0)
            show_stallguard_result (bool): prints StallGuard Result during movement
                (Default value = False)
            show_tstep (bool): prints TStep during movement (Default value = False)

        Returns:
            stop (enum): how the movement was finished
        """
        self._stop = self.NO
        current_vactual = 0
        sleeptime = 0.05
        time_to_stop = 0
        if vactual<0:
            acceleration = -acceleration

        if duration != 0:
            print(f"vactual: {vactual} for {duration} sec")
        else:
            print(f"vactual: {vactual}")
        print(str(bin(vactual)))

        print("writing vactual")
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


    def set_movement_abs_rel(self, movement_abs_rel):
        """set whether the movement should be relative or absolute by default.
        See the Enum MovementAbsoluteRelative

        Args:
            movement_abs_rel (enum): whether the movement should be relative or absolute
        """
        self._movement_abs_rel = movement_abs_rel



    def get_current_position(self):
        """returns the current motor position in µsteps

        Returns:
            int: current motor position
        """
        return self._current_pos



    def set_current_position(self, new_pos):
        """overwrites the current motor position in µsteps

        Args:
            new_pos (int): new position of the motor in µsteps
        """
        self._current_pos = new_pos


    def set_speed(self, speed):
        """sets the motor speed in steps per second

        Args:
            speed (int): speed in steps/sec
        """
        if speed == self._speed:
            return
        speed = self.constrain(speed, -self._max_speed, self._max_speed)
        if speed == 0.0:
            self._step_interval = 0
        else:
            self._step_interval = abs(1000000.0 / speed)
            if speed > 0:
                self.set_direction_pin_or_reg(1)
                print("going CW")
            else:
                self.set_direction_pin_or_reg(0)
                print("going CCW")
        self._speed = speed


    def set_speed_fullstep(self, speed):
        """sets the motor speed in fullsteps per second

        Args:
            speed (int): speed in fullsteps/sec
        """
        self.set_speed(speed*self.get_microstepping_resolution())


    def set_max_speed(self, speed):
        """sets the maximum motor speed in µsteps per second

        Args:
            speed (int): speed in µsteps per second
        """
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



    def set_max_speed_fullstep(self, speed):
        """sets the maximum motor speed in fullsteps per second

        Args:
            speed (int): maximum speed in fullsteps/sec
        """
        self.set_max_speed(speed*self.get_microstepping_resolution())



    def get_max_speed(self):
        """returns the maximum motor speed in steps per second

        Returns:
            int: current maximum speed in steps/sec
        """
        return self._max_speed



    def set_acceleration(self, acceleration):
        """sets the motor acceleration/deceleration in µsteps per sec per sec

        Args:
            acceleration (int): acceleration/deceleration in µsteps per sec per sec
        """
        if acceleration == 0.0:
            return
        acceleration = abs(acceleration)
        if self._acceleration != acceleration:
            # Recompute _n per Equation 17
            self._n = self._n * (self._acceleration / acceleration)
            # New c0 per Equation 7, with correction per Equation 15
            self._c0 = 0.676 * math.sqrt(2.0 / acceleration) * 1000000.0 # Equation 15
            self._acceleration = acceleration
            self.compute_new_speed()



    def set_acceleration_fullstep(self, acceleration):
        """sets the motor acceleration/deceleration in fullsteps per sec per sec

        Args:
            acceleration (int): acceleration/deceleration in fullsteps per sec per sec
        """
        self.set_acceleration(acceleration*self.get_microstepping_resolution())



    def get_acceleration(self):
        """returns the motor acceleration/deceleration in steps per sec per sec

        Returns:
            int: acceleration/deceleration in µsteps per sec per sec
        """
        return self._acceleration



    def stop(self, stop_mode = StopMode.HARDSTOP):
        """stop the current movement

        Args:
            stop_mode (enum): whether the movement should be stopped immediately or softly
                (Default value = StopMode.HARDSTOP)
        """
        self._stop = stop_mode



    def get_movement_phase(self):
        """return the current Movement Phase

        Returns:
            movement_phase (enum): current Movement Phase
        """
        return self._movement_phase

    def run_to_position_steps(self, steps, movement_abs_rel = None):
        """runs the motor to the given position.
        with acceleration and deceleration
        blocks the code until finished or stopped from a different thread!
        returns true when the movement if finished normally and false,
        when the movement was stopped

        Args:
            steps (int): amount of steps; can be negative
            movement_abs_rel (enum): whether the movement should be absolut or relative
                (Default value = None)

        Returns:
            stop (enum): how the movement was finished
        """
        if movement_abs_rel is None:
            movement_abs_rel = self._movement_abs_rel

        if movement_abs_rel == MovementAbsRel.RELATIVE:
            self._target_pos = self._current_pos + steps
        else:
            self._target_pos = steps

        self._stop = StopMode.NO
        self._step_interval = 0
        self._speed = 0.0
        self._n = 0
        self.compute_new_speed()
        while self.run(): #returns false, when target position is reached
            if self._stop == StopMode.HARDSTOP:
                break

        self._movement_phase = MovementPhase.STANDSTILL
        return self._stop

    def run(self):
        """calculates a new speed if a speed was made

        returns true if the target position is reached
        should not be called from outside!
        """
        if self.run_speed(): #returns true, when a step is made
            self.compute_new_speed()
        return self._speed != 0.0 and self.distance_to_go() != 0



    def distance_to_go(self):
        """returns the remaining distance the motor should run"""
        return self._target_pos - self._current_pos



    def compute_new_speed(self):
        """returns the calculated current speed depending on the acceleration

        this code is based on:
        "Generate stepper-motor speed profiles in real time" by David Austin
        https://www.embedded.com/generate-stepper-motor-speed-profiles-in-real-time/
        https://web.archive.org/web/20140705143928/http://fab.cba.mit.edu/classes/MIT/961.09/projects/i0/Stepper_Motor_Speed_Profile.pdf
        """
        distance_to = self.distance_to_go() # +ve is clockwise from current location
        steps_to_stop = (self._speed * self._speed) / (2.0 * self._acceleration) # Equation 16
        if ((distance_to == 0 and steps_to_stop <= 2) or
        (self._stop == StopMode.SOFTSTOP and steps_to_stop <= 1)):
            # We are at the target and its time to stop
            self._step_interval = 0
            self._speed = 0.0
            self._n = 0
            self._movement_phase = MovementPhase.STANDSTILL
            print("time to stop")
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
            self._pin_step.value=False
            if distance_to > 0:
                self.set_direction_pin_or_reg(1)
                print("going CW")
            else:
                self.set_direction_pin_or_reg(0)
                print("going CCW")
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



    def run_speed(self):
        """this methods does the actual steps with the current speed"""
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



    def make_a_step(self):
        """method that makes on step

        for the TMC2209 there needs to be a signal duration of minimum 100 ns
        """
        self._pin_step.value=True
        sleep_ns(1000)
        self._pin_step.value=False
        sleep_ns(1000)

        #print("one step")


    def rps_to_vactual(self, rps, steps_per_rev, fclk = 12000000):
        """converts rps -> vactual

        Args:
            rps (float): revolutions per second
            steps_per_rev (int): steps per revolution
            fclk (int): clock speed of the tmc (Default value = 12000000)

        Returns:
            vactual (int): value for vactual
        """
        return int(round(rps / (fclk / 16777216) * steps_per_rev))


    def vactual_to_rps(self, vactual, steps_per_rev, fclk = 12000000):
        """converts vactual -> rps

        Args:
            vactual (int): value for VACTUAL
            steps_per_rev (int): steps per revolution
            fclk (int): clock speed of the tmc (Default value = 12000000)

        Returns:
            rps (float): revolutions per second
        """
        return vactual * (fclk / 16777216) / steps_per_rev


    def rps_to_steps(self, rps, steps_per_rev):
        """converts rps -> steps/second

        Args:
            rps (float): revolutions per second
            steps_per_rev (int): steps per revolution

        Returns:
            steps (int): steps per second
        """
        return rps * steps_per_rev

    def steps_to_rps(self, steps, steps_per_rev):
        """converts steps/second -> rps

        Args:
            steps (int): speed in steps per second
            steps_per_rev (int): steps per revolution

        Returns:
            rps (float): revolutions per second
        """
        return steps / steps_per_rev


    def rps_to_tstep(self, rps, steps_per_rev, msres):
        """converts rps -> tstep

        Args:
            rps (float): revolutions per second
            steps_per_rev (int): steps per revolution
            msres (int): µstep resolution

        Returns:
            tstep (int): time per step
        """
        return int(round(12000000 / (rps_to_steps(rps, steps_per_rev) * 256 / msres)))


    def steps_to_tstep(self, steps, msres):
        """converts steps/second -> tstep

        Args:
            steps (int): speed in steps per second
            msres (int): µstep resolution

        Returns:
            tstep (int): time per step
        """
        return int(round(12000000 / (steps * 256 / msres)))


    def constrain(self, val, min_val, max_val):
        """constrains a value between a min and a max

        Args:
            val (int): value that should be constrained
            min_val (int): minimum value
            max_val (int): maximum value

        Returns:
            int: constrained value
        """
        if val < min_val:
            return min_val
        if val > max_val:
            return max_val
        return val
