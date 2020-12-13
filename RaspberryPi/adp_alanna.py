
class ADP:
    def __init__(self, controllers, seq_len, model_filename):
        self.nstates = 4
        self.seq_len = seq_len
        self.Ks = []
        for i in range(len(controllers)):
            self.Ks.append(np.loadtxt('K' + str(controllers[i]) + '.txt').reshape((1,4)))
        self.model = Model(nstates=self.nstates, noutputs=len(self.Ks), seq_len=self.seq_len)
        self.model.load_state_dict(model_filename)
        self.data_seq_list = []
    
    def main(s, b, d):
        print("Printing from ADP")
        print(d[:-1])
        self.data_seq_list = [d.reshape((1,self.nstates)))] + self.data_seq_list
        if (len(self.data_seq_list) > self.seq_len):
            self.data_seq_list.pop()
            self.data_seq = np.stack(self.data_seq_list, axis=0)

        out = self.model(self.data_seq)
        pred = torch.argmax(out, dim=1)

        self.K = self.Ks[pred]

        del self.data_seq
        del out
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
        x = x.unsqueeze(1)
        out = self.conv1(x)
        out = self.bn1(out)
        out = self.relu1(out)
        out = self.drop1(out)
        out = self.conv2(out)
        out = self.bn2(out)
        out = self.relu2(out)
        out = self.drop2(out)
        out = self.flat(out)
        out = self.fc1(out)
        out = self.fc2(out)
        return out