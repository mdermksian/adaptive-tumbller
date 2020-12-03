import sys
import time
import pigpio

import rls
import adp


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

    def attach_callback(self):
        """Attaches callback with pi as slave"""
        def cb(id, tick):
            print("I'm being called")
            s, b, d = self._pi.bsc_i2c(self.I2C_ADDR)
            if b:
                print(s)
                print(b)
                print(d[:-1])
        self._e = self._pi.event_callback(pigpio.EVENT_BSC, cb)

    def detach_callback(self):
        if self._e:
            self._e.cancel()

    def run(self):
        """Main function"""
        self.pigpio_setup()

        if self.mode == "RLS":
            print("Running in mode", self.mode)
            self.attach_callback()
            time.sleep(60)
            self.detach_callback()
        elif self.mode == "ADP":
            print("Running in mode", self.mode)
            self.attach_callback()
            time.sleep(60)
            self.detach_callback()
        else:
            print("Invalid mode")
            exit()

        self.pigpio_cleanup()

if __name__ == "__main__":
    if len(sys.argv) > 1:
        main = Main(sys.argv[1])
        main.run()
    else:
        print("Please supply the mode to run in")