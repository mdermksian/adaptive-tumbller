import pigpio
import struct
# import numpy
# from control import lqr
# import rls
import RPi.GPIO as GPIO
from time import sleep

SDA = 18
SCL = 19
I2C_ADDR = 69

def i2c_rls(id, tick):
    GPIO.output(20, GPIO.LOW)
#     global pi
#     global RLS
    s, b, d = pi.bsc_i2c(I2C_ADDR)
#     K = None
#     if b==16 and not RLS.state:
#         data = struct.unpack('ffff', d)
#         data = list(data)
#         print("Incoming:", data)
#         RLS.state.extend(data)
#     if b==2 and RLS.control is None:
#         data = struct.unpack('h', d)
#         data = data[0]
#         print("Incoming:", data)
#         RLS.control = data
#     if RLS.state and RLS.control is not None:
#         K = RLS.main(RLS.state, RLS.control)
#         print("Outgoing:", K)
#         out = struct.pack('ffff', K[0], K[1], K[2], K[3])
#         pi.bsc_i2c(I2C_ADDR, out)
#         RLS.state.clear()
#         RLS.control = None
    GPIO.output(20, GPIO.HIGH)

pi = pigpio.pi()
# RLS = rls.RLS()

GPIO.setmode(GPIO.BCM)
GPIO.setup(20, GPIO.OUT)
GPIO.output(20, GPIO.HIGH)

if not pi.connected:
    exit()

pi.set_pull_up_down(SDA, pigpio.PUD_UP)
pi.set_pull_up_down(SCL, pigpio.PUD_UP)

e = pi.event_callback(pigpio.EVENT_BSC, i2c_rls)

pi.bsc_i2c(I2C_ADDR)

print('Running')

try:
#     while True:
#         pass
    sleep(3600)
except KeyboardInterrupt:
    pass

e.cancel()

pi.bsc_i2c(0)

pi.stop()

GPIO.cleanup()
