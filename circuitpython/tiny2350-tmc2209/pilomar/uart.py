# Based on https://github.com/Short-bus/pilomar
# Simplified for use with TINY2350

import busio
import board
import time
import gc

from pilomar.helpers import *

#-----------------------------------------------------------------------------------------------
class uarthost():
    """ UART serial communication handler.
        Handles buffering of received and transmitted data over serial line. """
    def __init__(self, LogFile, ExceptionCounter, StatusLed, name='UART0', version=1.0):
        
        self.LogFile = LogFile
        self.ExceptionCounter = ExceptionCounter
        self.StatusLed = StatusLed
        self.version = version

        self.Clock = None
        
        self.uart = busio.UART(board.GP0,board.GP1,baudrate=115200,receiver_buffer_size=1024,timeout=0) # Define UART0 as the serial comms channel to the host.
        print ('UART TX=', board.GP0, 'UART RX=', board.GP1)
        self.WriteChunk = 32 # 32 seems OK on Circuitpython.
        self.ReceivedLines = [] # No lines received yet.
        self.LinesRead = 0 # total number of lines received.
        self.CharactersRead = 0 # Total number of characters received.
        self.LinesWritten = 0 # Total number of lines sent.
        self.PicoRxErrors = 0 # How many checksum rejections occurred with received data?
        self.CharactersWritten = 0 # total number of characters sent.
        self.StartTime = time.time()
        self.WriteGapms = 100 # ms pause between each chunk of data written.
        self.Name = name
        self.ReceivingLine = '' # Current line being received. It's constructed here until '\n' received.
        self.WriteQueue = [] # List of queued messages to be sent when safe.
        self.WriteDrops = 0 # Number of messages dropped because queue filled.
        self.ReadDrops = 0 # How many received messages are dropped because input buffer is full?
        self.LastTxms = self.LastRxms = self.ticks_ms() # Milliseconds since last transmission.
        print (self.Name, self.uart)

    def Reset(self):
        """ Reset communications (flush output buffers). """
        self.WriteQueue = [] # Empty the write queue.
        self.ReceivedLines = [] # Empty the input queue.
        self.ExceptionCounter.Reset() # Reset the ExceptionCounter also.
        for i in range(2): self.Write('#' * 20) # Send dummy lines through the UART line to flush out any junk.
        self.Write('# CP mem alloc ' + str(gc.mem_alloc()) + ' free ' + str(gc.mem_free()))
        self.Write('controller started') # Tell the remote device we're up and running.
        self.Write('controller version ' + str(self.version)) # Tell the remote device which software version is running.

    def ticks_ms(self):
        """ Standardise result of CircuitPython and MicroPython CPU ticks value. """
        return int(time.monotonic_ns() / 1e6) # Nano seconds. Reduce by 1.0e6 to get milliseconds (as integer).

    def CalculateChecksum(self,line):
        """ Simple checksum calculation. """
        cs = ""
        a = 0
        if len(line) > 0:
            for i in range(len(line)):
                if i % 2 == 0: a += ord(line[i])
                else: a += ord(line[i]) * 3
        cs = str(hex(a % 65536))[2:]
        return cs

    def AddChecksum(self,line):
        return line + '|' + self.CalculateChecksum(line)

    def RemoveChecksum(self,line):
        if '|' in line:
            l = line.split('|')
            line = l[0]
        return line

    def ValidateChecksum(self,line):
        """ Returns FALSE if checksum MISSING or INVALID.
            Returns TRUE if checksum EXISTS and is VALID. """
        result = False
        if '|' in line:
            l = line.split('|')
            if l[1] == self.CalculateChecksum(l[0]):
                result = True
        return result

    def RxWaiting(self):
        """ Return True if something in the Rx buffer.
            Standardises CircuitPython and MicroPython methods. """
        result = False
        if self.uart.in_waiting > 0: result = True
        return result

    def BufferInput(self):
        """ Read of UART serial port. For circuitpython 8 and later.
            Completed lines are added to the ReceivedLines list and
            are available to the rest of the program. """
        LoopCounter = 0
        while self.RxWaiting(): # Input waiting in Rx buffer.
            self.StatusLed.Task('coms')
            LoopCounter += 1
            CharsToProcess = '' # No characters to process yet.
            if LoopCounter > 20: break # Max 20 reads performed per call.
            try:
                bchar = self.uart.read() # Read entire waiting queue.
                CharsToProcess = ''.join([chr(b) for b in bchar]) # Convert to string.
                self.CharactersRead += len(CharsToProcess) # Count characters read.
            except Exception as e:
                self.LogFile.Log('uarthost.BufferInput: uart.read() or conversion error:', str(e))
                self.ExceptionCounter.Raise() # Increment exception count for the session.
            # Process each new character in turn.
            if len(CharsToProcess) > 0:
                for cchar in CharsToProcess:
                    self.ReceivingLine += cchar
                    if cchar == '\n': # End of line
                        self.LinesRead += 1
                        if len(self.ReceivingLine) > 0 and self.ReceivingLine[-1] == '\n':
                            line = self.ReceivingLine.strip() # Clear special characters.
                            if len(line) > 0: # Something to process.
                                if len(self.ReceivedLines) < 10: # Only buffer 10 lines, discard the rest. No space!
                                    self.ReceivedLines.append(line) # Add to list of lines to handle.
                                    line = self.RemoveChecksum(line)
                                    report = 'rec: ' + line
                                    print (report) # Report all receipts to serial out.
                                    if line[0] != "#": # Acknowledge receipt of all messages except comments via the log file back to the RPi too.
                                        x = line.split(' ')[-1] # Last entry should be message sequence number.
                                        if x.startswith('['): report = 'rec: ' + x # If we have message sequence number, just report that back to the RPi.
                                        self.LogFile.Log(report)
                                else:
                                    print('uarthost.BufferInput full. Ignored: ' + line)
                                    self.ReadDrops += 1
                        self.ReceivingLine = '' # Start a new receiving line with the next character received.
            self.LastRxms = self.ticks_ms() # When was last message received?
        self.StatusLed.Task('idle')

    def ReceiveAge(self):
        """ How many ms old is the last receipt? """
        return self.ticks_ms() - self.LastRxms # How old is the last receipt?
        # return LastRecMs

    def RemoveCounter(self,line):
        """ If the line ends with a message counter ('[nnn]') remove it. """
        temp = line.rfind('[')
        if temp >= 0:
            line = line[:temp].strip() # Strip anything after the last '[' character, it's a message count and not data.
        return line

    def Read(self):
        """ Return next available complete received line. """
        # 1st check for new data received on serial port.
        self.BufferInput() # Check UART port.
        line = ''
        while len(line) == 0 and len(self.ReceivedLines) > 0:
            line = self.ReceivedLines.pop(0)
            if self.ValidateChecksum(line): # Checksum is good, trust the line.
                line = self.RemoveChecksum(line)
                line = self.RemoveCounter(line) # Strip any final message count from the end of the line.
            else: # Checksum is bad, reject the line.
                print ('uarthost.Read: Rejected checksum : ' + line)
                self.LogFile.Log('uarthost.Read: Rejected checksum : ' + line)
                self.PicoRxErrors += 1
                line = ''
        return line

    def WritePoll(self):
        """ Write queued lines to the serial port.
            Clear inbound characters received first! """
        if self.RxWaiting(): # Input waiting to be handled is higher priority.
            return # Something waitint to receive takes priority.
        LastSendMs = self.ticks_ms() - self.LastTxms
        if LastSendMs < self.WriteGapms:
            return # 200ms needed between each transmission.
        if LastSendMs > 30000: # After 30 seconds of silence, send a heartbeat signal.
            self.Heartbeat()
        if len(self.WriteQueue) == 0:
            return # Nothing to send.
        self.StatusLed.Task('coms')
        if len(self.WriteQueue[0]) > self.WriteChunk: # Pull max 20 chars from write queue.
            line = self.WriteQueue[0][:self.WriteChunk]
            self.WriteQueue[0] = self.WriteQueue[0][self.WriteChunk:]
        else: # Pull the whole remaining line from the queue.
            line = self.WriteQueue.pop(0) + "\n"
        if len(line.strip()) == 0:
            print ('uarthost.WritePoll: ignored null line in WriteQueue.')
        byteline = line.encode('utf-8') # Convert to bytearray.
        self.uart.write(byteline) # Physical write.
        if line[-1] == "\n": self.LinesWritten += 1
        self.CharactersWritten += len(line)
        self.LastTxms = self.ticks_ms()
        self.StatusLed.Task('idle')

    def Write(self,line,log=True):
        # Add a line to the write queue. It's physically sent separately by WritePoll()
        # It queues a limited number of messages for sending.
        # After that, the queue only accepts extra messages if force==True.
        # Most communication is self-recovering, so it doesn't usually matter if we have to abandon a
        # message, the message will be raised gain soon.
        line = line.strip() # Clean the line.
        if len(line) > 0:
            while len(self.WriteQueue) >= 20: # Only buffer 20 lines. Save memory.
                self.WriteQueue.pop(1) # Drop the 2nd entry, the first may already be partially transmitted.
                self.WriteDrops += 1
            self.WriteQueue.append(self.AddChecksum(line))
            print ('controller queueing: ' + line)

    def Heartbeat(self):
        """ Send Heartbeat signal to the RPi. """
        self.Write('controller heartbeat ' + IntToTimeString(self.Clock.Now()) + " on " + IntToTimeString(time.time()))
