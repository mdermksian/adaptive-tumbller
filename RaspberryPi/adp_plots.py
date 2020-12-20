import numpy as np
import matplotlib.pyplot as plt
f = open("/home/pi/adaptive-tumbller/RaspberryPi/predictions_model_9_adp_collect_data","r")
# print(f.read())
data = np.genfromtxt(f, delimiter=',')
print('data shape = ',data.shape)

# data = data[:156]
# x = np.split(data,4)
# x = np.array(x)
# print(x.shape)
# split_mean = np.mean(x,axis=0)
# 
# plt.figure()
# plt.plot( [round(num) for num in split_mean]    )
# plt.ylabel('Average Prediction Label')
# plt.xlabel('Time')
# plt.title('Predicitions vs. Time')
# plt.show()
reverse_array = data[::-1]


plt.figure()
plt.plot( data   )
plt.ylabel('States')
plt.xlabel('Time Steps')
plt.title('States over Time w/ ADP')
plt.legend(['x','xdot','theta','theta dot'])
plt.ylim(-5,5)
plt.show()

# plt.figure()
# plt.plot( reverse_array   )
# plt.ylabel('Prediction Label')
# plt.xlabel('Time Steps')
# plt.title('Predicitions vs. Time Steps')
# plt.show()
# test_list = [-1,-2,3]
# print(test_list[-1])