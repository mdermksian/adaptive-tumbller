import sys
import serial
import struct
import time
from rls import RLS
from adp_alanna import ADP

class Main:
    BAUD = 57600
    
    def __init__(self, mode):
        if mode == "RLS":
            self.incoming_size = 14
            self.decode_string = 'fffh'
            self.program = RLS()
            print('Running RLS')
        elif mode == "ADP":
            self.incoming_size = 16
            self.decode_string = 'ffff'
            self.program = ADP()
        else:
            print("Please input a valid run mode")
            exit()
        self.mode = mode
        self.cnt = 0
        self.ser = serial.Serial('/dev/ttyUSB0', self.BAUD, timeout=0.01) # <-- potentially address issues with timeout
        self.ser.flush()
    
    def run_rls(self):
        try:
            while True:
                if self.ser.in_waiting > 0:
                    incoming = self.ser.read(self.incoming_size)
                    if len(incoming) != self.incoming_size:
                        continue;
                    inp = struct.unpack(self.decode_string, incoming)
#                    print("Incoming:", inp)
                    K = self.program.main(inp, self.cnt)
                    self.cnt += 1
                    print(self.cnt)
#                    print("Outgoing:", K)
                    outgoing = struct.pack("ffff", K[0], K[1], K[2], K[3])
                    self.ser.write(outgoing)
        except KeyboardInterrupt:
            pass
        
        self.program.cleanup()
        print('Finishing')
    
    def run_adp(self):
        try:
            while True:
                if self.ser.in_waiting > 0:
                    incoming = self.ser.read(self.incoming_size)
                    if len(incoming) != self.incoming_size:
                        continue;
                    inp = struct.unpack(self.decode_string, incoming)
                    K = self.program.main(inp)
                    outgoing = struct.pack("f", K)
                    self.ser.write(outgoing)
        except KeyboardInterrupt:
            pass
        self.program.cleanup()

    def run(self):
        if self.mode == "ADP":
            self.run_adp()
        elif self.mode == "RLS":
            self.run_rls()

        
if __name__ == "__main__":
    if len(sys.argv) > 1:
        main = Main(sys.argv[1])
        main.run()
    else:
        print("Please supply the mode to run in")
