import sys
import serial
import struct
import time
from rls import RLS
from adp_data_collect import ADP

class Main:
    BAUD = 57600
    
    def __init__(self, mode):
        if mode == "RLS":
            self.incoming_size = 14
            self.decode_string = 'fffh'
            self.program = RLS()
        elif mode == "ADP":
            self.incoming_size = 16
            self.decode_string = 'ffff'
            self.program = ADP()
        else:
            print("Please input a valid run mode")
            exit()
        self.cnt = 0
        self.ser = serial.Serial('/dev/ttyUSB0', self.BAUD, timeout=0.01) # <-- potentially address issues with timeout
        self.ser.flush()

    def run(self):
        while True:
            if self.ser.in_waiting > 0:
                incoming = self.ser.read(self.incoming_size)
                if len(incoming) != self.incoming_size:
                    continue;
                inp = struct.unpack(self.decode_string, incoming)
                print("Incoming:", inp)
                K = self.program.main(inp, self.cnt)
                self.cnt += 1
                print("Outgoing:", K)
                outgoing = struct.pack("ffff", K[0], K[1], K[2], K[3])
                self.ser.write(outgoing)

        
if __name__ == "__main__":
    if len(sys.argv) > 1:
        main = Main(sys.argv[1])
        main.run()
    else:
        print("Please supply the mode to run in")
