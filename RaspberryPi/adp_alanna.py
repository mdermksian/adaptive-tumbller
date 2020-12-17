import torch
import torch.nn as nn
import numpy as np
class ADP:
    def __init__(self):
        self.nstates = 4
        self.seq_len = 500
        self.Ks = []
        # for i in range(len(controllers)):
        #     self.Ks.append(np.loadtxt('K' + str(controllers[i]) + '.txt').reshape((1,4)))
        self.Ks.append(np.array((-0.00288150139069816,-0.108593947376385,26.3953358864335,1.88977789448912) ) )    # K0 (no weights)
        self.Ks.append(np.array((-0.00289199476972477,-0.109318875767478,26.7396667177963,1.95468611771592) ) )    # K2
        self.Ks.append(np.array((-0.00290101208252616,-0.10999844278361,27.0779262509664,2.01712436203214) ) )     # K4
        self.Ks.append(np.array((-0.00290881495680823,-0.110640598087607,27.4112553504243,2.07718961558237) ) )     # K6


        self.model = Model(nstates=self.nstates, noutputs=len(self.Ks), seq_len=self.seq_len)
        self.model.load_state_dict(torch.load('model.pt'))

        self.all_states = []
        self.model_input = []
    
    def main(self,state):
        #print("Printing from ADP")
        state = np.array(state)
        self.all_states.append(state.reshape((1,self.nstates)))
        print(np.array(self.all_states).shape)
        pred = 0
        # print(d[:-1])
        # self.data_seq_list = [d.reshape((1,self.nstates)))] + self.data_seq_list
        if (len(self.all_states) > 10):
            self.all_states.pop(0)
            data_seq = np.stack(self.all_states, axis=2)
            print(data_seq.shape)
            out = self.model(data_seq)
            pred = torch.argmax(out, dim=1)
            del data_seq
            del out

        self.K = self.Ks[pred]
        
            
        del pred

        return self.K

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
        print(out.shape)
        out = self.flat(out)
        print(out.shape)
        out = self.fc1(out)
        out = self.fc2(out)
        return out