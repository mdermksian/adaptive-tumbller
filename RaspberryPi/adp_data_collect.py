import numpy as np

class ADP:

    def __init__(self):
        self.all_states = []
        self.model_input = []
        self.collect_data = []
        self.counter = 0

    def main(self, state):
        # print("Printing from ADP")
        # print('state = ', state)
        self.counter += 1
        print(self.counter)
        
        self.all_states.append(state)
        self.collect_data.append(state)

        if (np.array(self.collect_data).shape[0] >= 8000):
            self.collect_data = np.array(self.collect_data)
            np.savetxt('weight0',self.collect_data,delimiter = ',')

        # # print(all_dem_states)
        # if ((np.array(self.all_states)).shape[0] >= 500):
        #     self._all_states = np.array(self.all_states)

        #     model_input = self_all_states[:500,:]
        #     self.all_states = np.delete(self.all_states,np.s_[0], axis=0)

        #     self.all_states = np.ndarray.tolist(self.all_states)
        #     # print(model_input)
            # print(model_input.shape)

        K = state
        return K