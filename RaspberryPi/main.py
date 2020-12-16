import sys
import time
import struct
import pigpio

from rls import RLS
# from adp import ADP
from adp_data_collect import ADP


def debug_helper(s, b, first):
    print((first + ':\t' + str(b) + '\t' + str(s>>16) + '\t' + str((s>>11) & 0x1f) + '\t' + \
          str((s>>6) & 0x1f) + '\t' + '{0:06b}').format((s & 0x3f)))


class Main:
    """Main program singleton class"""

    SDA = 18
    SCL = 19
    I2C_ADDR = 69
    _e = None
    
    cnt = 0
    
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
        
    def cb_rls(self, id, tick):
        s, b, d = self._pi.bsc_i2c(self.I2C_ADDR)
#         debug_helper(s, b, '1')
        if(b == 14):
            data = struct.unpack('fffh', d)
            K = self.rls.main(data, self.cnt)
            out = struct.pack('ffff', K[0], K[1], K[2], K[3])
            self._pi.bsc_i2c(self.I2C_ADDR, out)
#             s, b, d = self._pi.bsc_i2c(self.I2C_ADDR, out)
#            print(self.cnt)
            self.cnt += 1
#             debug_helper(s, b, '2')
    
    def cb_adp(self, id, tick):
        print("ADP")
        s, b, d = self._pi.bsc_i2c(self.I2C_ADDR)
        if b == 16:
            data = struct.unpack('ffff', d)
            K = self.adp.main(data)
            out = struct.pack('ffff', K[0], K[1], K[2], K[3])
            self._pi.bsc_i2c(self.I2C_ADDR, out)
            
    def attach_callback(self, fun):
        self._e = self._pi.event_callback(pigpio.EVENT_BSC, fun)
        self._pi.bsc_i2c(self.I2C_ADDR)

    def detach_callback(self):
        if self._e:
            self._e.cancel()

    def run(self):
        """Main function"""
        self.pigpio_setup()

        if self.mode == "RLS":
            self.rls = RLS()
            self.attach_callback(self.cb_rls)
            print("Running in mode", self.mode)
        elif self.mode == "ADP":
            self._expected = 16
            self._received = 0
            self.adp = ADP()
            self.attach_callback(self.cb_adp)
            print("Running in mode", self.mode)
        else:
            print("Invalid mode")
            exit()
            
        try:
            time.sleep(3600) # 1 hour
        except KeyboardInterrupt:
            self.detach_callback()

        self.pigpio_cleanup()

if __name__ == "__main__":
    if len(sys.argv) > 1:
        main = Main(sys.argv[1])
        main.run()
    else:
        print("Please supply the mode to run in")
