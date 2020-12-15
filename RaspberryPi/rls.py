import numpy as np
from control import lqr

class RLS:
    def __init__(self):
        self.state = []
        self.control = None
        
        self.mcart = 0.493
        self.mpend = 0.312
        self.Ipend = 0.00024
        self.L = 0.04
        self.f = 0.01
        self.kT = 0.11
        self.R = 10
        self.r = 0.0335
        self.g = 9.81
        
#         self.Qmat = np.diag([0.01, 0.01, 1000, 1]) # Mike's
#         self.Rmat = 0.1
#         self.Qmat = np.diag([1, 1, 8e7, 3e4]) # Sam's
#         self.Rmat = 2
        self.Qmat = np.diag([1, 1, 8e7, 3e4])
        self.Rmat = 2
        self.computeAB()
        self.initRLS()
        
        self.Tsamp = 0.015 # sampling time (s)
        
        self.state = np.ndarray((4, 1))
        self.state_pre = np.ndarray((4, 1))
        self.ctrl_pre = 0;
    
    
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
        ]) * 2*self.kT/(self.R*self.r) * 255/8.0
    
    
    def initRLS(self):
        self.theta = np.ndarray((6, 1))
        self.theta[0, 0] = self.A[1, 1]
        self.theta[1, 0] = self.A[1, 2]
        self.theta[2, 0] = self.A[3, 1]
        self.theta[3, 0] = self.A[3, 2]
        self.theta[4, 0] = self.B[1, 0]
        self.theta[5, 0] = self.B[3, 0]
        
        p0 = 1e8
        self.P = p0 * np.identity(6)
        
        self.forget = 1.0
        
        self.phi = np.zeros((6, 4))
        self.delta = np.ndarray((4, 1))
        
        self.first = True
    
    
    def updateRLS(self):
        self.phi[0:2, 1] = self.Tsamp * self.state_pre[1:3, 0]
        self.phi[4, 1] = self.Tsamp * self.ctrl_pre
        self.phi[2:4, 3] = self.Tsamp * self.state_pre[1:3, 0]
        self.phi[5, 3] = self.Tsamp * self.ctrl_pre
        
        self.delta[0, 0] = self.state_pre[0, 0] + self.Tsamp * self.state_pre[1, 0]
        self.delta[1, 0] = self.state_pre[1, 0]
        self.delta[2, 0] = self.state_pre[2, 0] + self.Tsamp * self.state_pre[3, 0]
        self.delta[3, 0] = self.state_pre[3, 0]
        
        KN = self.P @ self.phi @ np.linalg.inv(self.forget * np.identity(4) + self.phi.T @ self.P @ self.phi)
        self.theta = self.theta + KN @ (self.state - self.phi.T @ self.theta - self.delta)
        self.P = (np.identity(6) - KN @ self.phi.T) @ self.P / self.forget
        
        self.updateModel()
    
    
    def updateModel(self):
        self.A[1, 1:3] = np.transpose(self.theta[0:2, 0])
        self.A[3, 1:3] = np.transpose(self.theta[2:4, 0])
        self.B[1, 0] = self.theta[4, 0]
        self.B[3, 0] = self.theta[5, 0]
    
    
    def computeLQR(self):
        K, _, _ = lqr(self.A, self.B, self.Qmat, self.Rmat)
        K = np.array(K)
        return K.ravel()
    
    
    def main(self, data):
        data_mat = np.array(data)
        
        self.state_pre = np.copy(self.state);
        
        self.state[1:4, 0] = data_mat[0:3]
        self.state[0, 0] = self.state_pre[0,0] + self.Tsamp * self.state[1,0]
#         self.ctrl_pre = data_mat[3] * 8.0 / 255.0 # convert PWM to volts
        self.ctrl_pre = data_mat[3]
        
#         print(data_mat[3])
        
        if(self.first):
            self.first = False
        else:
            self.updateRLS()
        
        K = self.computeLQR()
        K = K.tolist()
        return K # <--- This needs to be a python list of 4 floats
    