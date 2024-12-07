import board
import busio
import digitalio
import time

# TMC2209 registermap
GCONF = 0x00
GSTAT = 0x01
IFCNT = 0x02
SLAVECONF = 0x03
OTP_PROG = 0x04
OTP_READ = 0x05
IOIN = 0x06
FACTORY_CONF = 0x07
IHOLD_IRUN = 0x10
TPOWER_DOWN = 0x11
TSTEP = 0x12
TPWMTHRS = 0x13
TCOOLTHRS = 0x14
VACTUAL = 0x22
SGTHRS = 0x40
SG_RESULT = 0x41
COOLCONF = 0x42
MSCNT = 0x6A
MSCURACT = 0x6B
CHOPCONF = 0x6C
DRV_STATUS = 0x6F
PWMCONF = 0x70
PWM_SCALE = 0x71
PWM_AUTO = 0x72

uart = busio.UART(board.GP4, board.GP5, baudrate=115200)

def compute_crc8_atm(datagram, initial_value=0):
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

def read_reg(uart,mtr_id,reg):
    x = [0x55, 0, 0, 0]
    x[1] = mtr_id
    x[2] = reg
    x[3] = compute_crc8_atm(x[:-1])
    y = uart.write(bytes(x))
    if y != len(x):
        print("Err in read")
        return False
    time.sleep(.01)
    y = uart.read(4)
    if y:
        y = uart.read()
    else:
        y = 0
        time.sleep(.000005)
    return(y)

def write_reg(uart,mtr_id,reg,val):
    x = [0x55, 0, 0, 0, 0, 0, 0, 0]
    x[1] = mtr_id
    x[2] = reg | 0x80
    x[3] = 0xFF & (val>>24)
    x[4] = 0xFF & (val>>16)
    x[5] = 0xFF & (val>>8)
    x[6] = 0xFF & val
    x[7] = compute_crc8_atm(x[:-1])
    y = uart.write(bytes(x))
    if y != len(x):
        print("Err in write")
        return False
    time.sleep(.01)
    y = uart.read()#read what it self send and trash it (RX and TX one line)
    time.sleep(.000002)

z = 0
while True:
    z = z + 1
    a = read_reg(uart, 0, IOIN)
    print(a[3],a[4],a[5],a[6])
    print("##############################")
    a = read_reg(uart, 0, IFCNT)
    write_reg(uart, 0, GCONF, 0x000000020)
    b = read_reg(uart, 0, IFCNT)
    print("reading reg attempt", z)
    for i in bytearray(a):
        print("REG: ", hex(i))
        print("-")
    for i in bytearray(b):
        print("REG: ", hex(i))
        print("##############################")
    input("Go again?")
