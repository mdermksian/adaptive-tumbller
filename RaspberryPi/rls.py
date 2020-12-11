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
        
        self.Qmat = np.diag([0.01, 0.01, 1000, 1])
        self.Rmat = 0.1
        self.computeAB()
        self.initRLS()
        
        self.Tsamp = 0.01 # sampling time (s)
    
    
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
    
    
    def updateRLS(self, meas, ctrl):
        self.phi[0:2, 1] = self.Tsamp * meas[1:3, 0]
        self.phi[4, 1] = self.Tsamp * ctrl
        self.phi[2:4, 3] = self.Tsamp * meas[1:3, 0]
        self.phi[5, 3] = self.Tsamp * ctrl
        
        self.delta[0, 0] = meas[0, 0] + self.Tsamp * meas[1, 0]
        self.delta[1, 0] = meas[1, 0]
        self.delta[2, 0] = meas[2, 0] + self.Tsamp * meas[3, 0]
        self.delta[3, 0] = meas[3, 0]
        
        K = self.P @ self.phi @ np.linalg.inv(self.forget * np.identity(4) + self.phi.T @ self.P @ self.phi)
        self.theta = self.theta + K @ (meas - self.phi.T @ self.theta - self.delta)
        self.P = (np.identity(6) - K @ self.phi.T) @ self.P / self.forget
        
        self.updateModel()
    
    
    def updateModel(self):
        self.A[1, 1:3] = np.transpose(self.theta[0:2, 0])
        self.A[3, 1:3] = np.transpose(self.theta[2:4, 0])
        self.B[1, 0] = self.theta[4, 0]
        self.B[3, 0] = self.theta[5, 0]
    
    
    def computeLQR(self):
        K, S, E = lqr(self.A, self.B, self.Qmat, self.Rmat)
        K = np.array(K)
        return K.ravel()
    
    
    def main(self, state, ctrl):
        #data_mat = np.array(data)
        #state = np.expand_dims(data_mat[0:4], 1)
        #ctrl = data_mat[4]
        #print("Incoming:", data_mat)
        state = np.array(state).reshape(-1,1)
        self.updateRLS(state, ctrl)
        K = self.computeLQR()
        K = K.tolist()
        return K # <--- This needs to be a python list of 4 floats
    
