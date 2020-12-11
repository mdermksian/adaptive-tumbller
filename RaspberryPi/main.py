import sys
import time
import struct
import pigpio
import RPi.GPIO as GPIO

from rls import RLS
from adp import ADP


class Main:
    """Main program singleton class"""

    SDA = 18
    SCL = 19
    I2C_ADDR = 69
    _e = None
    
    def __init__(self, mode):
        """Constructor for the main program. Accepts the mode: RLS or ADP"""
        self._pi = pigpio.pi()
        self.mode = mode

    def pigpio_setup(self):
        """Performs setup of the gpio pins"""
        if not self._pi.connected:
            print("pi not connected")
            exit()

        self._pi.set_pull_up_down(self.SDA, pigpio.PUD_UP)
        self._pi.set_pull_up_down(self.SCL, pigpio.PUD_UP)

    def pigpio_cleanup(self):
        """Cleans up gpio activity"""
        self._pi.bsc_i2c(0)
        self._pi.stop()

    def attach_callback(self, fun):
        def cb(id, tick):
            GPIO.output(20, GPIO.LOW)
            if self.mode == 'RLS':
                s, b, d = self._pi.bsc_i2c(self.I2C_ADDR)
                if(b == 14):
                    data = struct.unpack('fffh', d)
                    K = fun(data)
                    out = struct.pack('ffff', K[0], K[1], K[2], K[3])
                    self._pi.bsc_i2c(self.I2C_ADDR, out)
#                 if b == 16 and not self._received_state:
#                     data = struct.unpack('ffff', d)
#                     print("Incoming:", data)
#                     self._received_state.extend(data)
#                 if b == 2 and not self._received_control:
#                     data = struct.unpack('h', d)
#                     print("Incoming:", data)
#                     self._received_control = data
#                 if self._recieved_state and self._received_control:
#                     K = fun(self._received_state, self._recieved_control)
#                     out = struct.pack('ffff', K[0], K[1], K[2], K[3])
#                     print("Outgoing:", out)
#                     self._pi.bsc_i2c(self.I2C_ADDR, out)
#                     self._recieved_state.clear()
#                     self._recieved_control.clear()
            else:
                print("ADP")
                s, b, d = self._pi.bsc_i2c(self.I2C_ADDR)
                if b == 16:
                    data = struct.unpack('ffff', d)
                    K = fun(data)
                    out = struct.pack('ffff', K[0], K[1], K[2], K[3])
                    self._pi.bsc_i2c(self.I2C_ADDR, out)
            GPIO.output(20, GPIO.HIGH)

        self._e = self._pi.event_callback(pigpio.EVENT_BSC, cb)
        self._pi.bsc_i2c(self.I2C_ADDR)

    def detach_callback(self):
        if self._e:
            self._e.cancel()

    def run(self):
        """Main function"""
        self.pigpio_setup()
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(20, GPIO.OUT)
        GPIO.output(20, GPIO.HIGH)

        if self.mode == "RLS":
            print("Running in mode", self.mode)
            self._received_state  = [] 
            self._received_conrol = []
            self.rls = RLS()
            self.attach_callback(self.rls.main)
        elif self.mode == "ADP":
            print("Running in mode", self.mode)
            self._expected = 16
            self._received = 0
            self.adp = ADP()
            self.attach_callback(self.adp.main)
        else:
            print("Invalid mode")
            exit()
            
        try:
            time.sleep(3600) # 1 hour
        except KeyboardInterrupt:
            self.detach_callback()

        self.pigpio_cleanup()
        GPIO.cleanup()

if __name__ == "__main__":
    if len(sys.argv) > 1:
        main = Main(sys.argv[1])
        main.run()
    else:
        print("Please supply the mode to run in")
