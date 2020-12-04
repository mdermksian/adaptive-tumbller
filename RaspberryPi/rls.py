import struct
import numpy as np
from control import lqr

class RLS:
    def __init__(self):
        self.mcart = 0.493
        self.mpend = 0.312
        self.Ipend = 0.00024
        self.L = 0.04
        self.f = 0.01
        self.kT = 0.11
        self.R = 10
        self.r = 0.0335
        self.g = 9.81
        
        self.Qmat = np.diag([0.01, 0.01, 1000, 1])
        self.Rmat = 0.1
        self.computeAB()
    
    def computeAB(self):
        denom = self.Ipend*(self.mcart+self.mpend)+self.mcart*self.mpend*self.L**2
        self.A = np.array([
            [0, 1, 0, 0],
            [0, -(self.Ipend+self.mpend*self.L**2)*self.f/denom, self.mpend**2*self.g*self.L**2/denom, 0],
            [0, 0, 0, 1],
            [0, -self.mpend*self.L*self.f/denom, self.mpend*self.g*self.L*(self.mcart+self.mpend)/denom, 0]
        ])
        
        self.B = np.array([
            [0],
            [(self.Ipend+self.mpend*self.L**2)/denom],
            [0],
            [self.mpend*self.L/denom]
        ])
    
    def computeLQR(self):
        K, S, E = lqr(self.A, self.B, self.Qmat, self.Rmat)
        K = np.array(K)
        return K.ravel()
    
    def main(self, s, b, d):
        floats = struct.unpack('ffff', d)
        float_mat = np.array(floats)
        print("Incoming:", float_mat)
        
        K = self.computeLQR()
        print("Outgoing:", K)
        test = K.tolist()
        return struct.pack('ffff', test[0], test[1], test[2], test[3])