import torch
import torch.nn as nn
import numpy as np
class ADP:
    def cleanup(self):
        np.savetxt('predictions_model_9_no_adp_collect_data',np.array(self.collect_data),delimiter = ',')
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
        self.Ks.append(np.array((-0.316227, -2.60198, 114.9365, 5.84731) ) )     # K8
        self.Ks.append(np.array((-0.316227766, -2.60280197, 115.1717, 6.03343) ) )     # K10
#         self.theta_ref = (np.array((0.0,0.0618389, 0.1200545, 0.174596, 0.17459613)))
        
        self.theta_ref = (np.array((0.01518, -0.022689, -0.023736, -0.051836, -0.06527,-0.075049)))
        
        self.model = Model(nstates=self.nstates, noutputs=3, seq_len=(self.seq_len))
        self.model.load_state_dict(torch.load('model_9.pt'))

        self.all_states = []
        self.model_input = []
        self.counter = 0
        self.pred_list = []
        self.collect_data = []
        
    def main(self,state):
        #print("Printing from ADP")
        state = np.array(state)
#         if (np.array(self.pred_list).size ==0):
#             state[2] -= self.theta_ref[0]
#         else:
#             state[2] -= self.theta_ref[self.pred_list[-1]]
#         self.K = self.Ks[0]
#         if self.counter > 3000:
#             self.K = self.Ks[2]
#             if self.counter > 6000:
#                 self.K = self.Ks[4]
#                 
#         state[2] -= self.theta_ref[4]
        
        self.all_states.append(state.reshape((1,self.nstates)))
        self.collect_data.append(state)

        
            

        pred = 0
        
        if (len(self.all_states) > self.seq_len):
            self.all_states.pop(0)
#             print(data_seq.shape)
            if self.counter >= 10:
                data_seq = np.stack(self.all_states, axis=2)
                out = self.model(data_seq)
                pred = torch.argmax(out, dim=1)[0]
                print(pred.numpy())
                self.pred_list.append(pred.numpy())
                del data_seq
                del out
                self.counter = 0
        
#         state[2] -= self.theta_ref[pred]
        self.K = self.Ks[0]
       
        
        
        del pred
        self.counter += 1
        u = self.K.reshape((-1))@state
        return u*255/8

class Model(nn.Module):
    def __init__(self, nstates, noutputs, seq_len):
        super(Model, self).__init__()
        self.conv1 = nn.Conv2d(in_channels=1, out_channels=8, kernel_size=(2,20), stride=1)
        self.bn1 = nn.BatchNorm2d(8, momentum=0.1)
        self.relu1 = nn.ReLU(inplace=True)
        self.drop1 = nn.Dropout(p=0.3)
        
        self.conv2 = nn.Conv2d(in_channels=8, out_channels=16, kernel_size=(2,20), stride=1)
        self.bn2 = nn.BatchNorm2d(16, momentum=0.1)
        self.relu2 = nn.ReLU(inplace=True)
        self.drop2 = nn.Dropout(p=0.3)
        
        self.flat = nn.Flatten()
        self.fc1 = nn.Linear(16*(nstates-2)*(seq_len-38), noutputs*64)
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