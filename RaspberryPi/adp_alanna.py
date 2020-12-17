import torch
import torch.nn as nn
import numpy as np
class ADP:
    def __init__(self):
        self.nstates = 4
        self.seq_len = 300
        self.Ks = []
        # for i in range(len(controllers)):
        #     self.Ks.append(np.loadtxt('K' + str(controllers[i]) + '.txt').reshape((1,4)))
#         self.Ks.append(np.array((-0.00288150139069816,-0.108593947376385,26.3953358864335,1.88977789448912) ) )    # K0 (no weights)
        self.Ks.append(np.array((-0.31622, -2.59802, 113.942, 4.8661) ) )    # K0 (no weights)
        self.Ks.append(np.array((-0.316227, -2.59913, 114.200, 5.1528) ) )    # K2
        self.Ks.append(np.array((-0.316227,-2.6001,114.451, 5.4098) ) )     # K4
        self.Ks.append(np.array((-0.316227,-2.6011,114.6967, 5.64036) ) )     # K6

#         self.theta_ref = (np.array((0.0,0.0618389, 0.1200545, 0.174596, 0.17459613)))
        
        self.theta_ref = (np.array((0.0,0.036, 0.0713, 0.1394, 0.17459613)))
        self.model = Model(nstates=self.nstates, noutputs=len(self.Ks), seq_len=self.seq_len)
        self.model.load_state_dict(torch.load('model_pi.pt'))

        self.all_states = []
        self.model_input = []
        self.counter = 0
        self.collect_data = []
        self.counter_2 = 0
    def main(self,state):
        #print("Printing from ADP")
        state = np.array(state)
#         state[2] += self.theta_ref[0]
        
        self.all_states.append(state.reshape((1,self.nstates)))
        self.K = self.Ks[0]
        self.collect_data.append(state)
#         print((np.array(self.collect_data)).shape[0])
        
#         if ((np.array(self.collect_data)).shape[0] >= 8000):
#             self.collect_data = np.array(self.collect_data)
#             np.savetxt('weight0',self.collect_data,delimiter = ',')
#             print('save successful')
#             self.collect_data = np.ndarray.tolist(self.collect_data)
            
#         print(np.array(self.all_states).shape)
        pred = 0

        if (len(self.all_states) > self.seq_len):
            self.all_states.pop(0)
#             print(data_seq.shape)
            if self.counter >= 10:
                data_seq = np.stack(self.all_states, axis=2)
                out = self.model(data_seq)
                pred = torch.argmax(out, dim=1)[0]
                pred = 0
                del data_seq
                del out
                self.counter = 0
#                 print(pred)

        self.K = self.Ks[0]
        
        
        del pred
        self.counter += 1

        u = self.K.reshape((-1))@state
#         print(u*255/8)
        return u*255/8

class Model(nn.Module):
    def __init__(self, nstates, noutputs, seq_len):
        super(Model, self).__init__()
        self.conv1 = nn.Conv2d(in_channels=1, out_channels=8, kernel_size=(2,5), stride=1)
        self.bn1 = nn.BatchNorm2d(8, momentum=0.1)
        self.relu1 = nn.ReLU(inplace=True)
        self.drop1 = nn.Dropout(p=0.2)
        self.conv2 = nn.Conv2d(in_channels=8, out_channels=16, kernel_size=(2,5), stride=1)
        self.bn2 = nn.BatchNorm2d(16, momentum=0.1)
        self.relu2 = nn.ReLU(inplace=True)
        self.drop2 = nn.Dropout(p=0.2)
        self.flat = nn.Flatten()
        self.fc1 = nn.Linear(16*(nstates-2)*(seq_len-8), noutputs*64)
        self.fc2 = nn.Linear(noutputs*64, noutputs)
    
    def forward(self, x):
        x = torch.FloatTensor(x)
        x = x.unsqueeze(1)
        out = self.conv1(x)
        out = self.bn1(out)
        out = self.relu1(out)
        out = self.drop1(out)
        out = self.conv2(out)
        out = self.bn2(out)
        out = self.relu2(out)
        out = self.drop2(out)
#         print(out.shape)
        out = self.flat(out)
#         print(out.shape)
        out = self.fc1(out)
        out = self.fc2(out)
        return out