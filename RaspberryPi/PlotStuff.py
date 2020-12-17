import numpy as np
import matplotlib.pyplot as plt

file = open('/home/pi/Desktop/RLS_data.csv', 'r')
data = np.genfromtxt(file, delimiter=',')

n_plots = 4
# n_plots = 6

# Plot 2x2
if(n_plots == 4):
    plt.figure()
    ax = plt.subplot(2, 2, 1)
    ax.plot(data[:,0])
    ax = plt.subplot(2, 2, 2)
    ax.plot(data[:,1])
    ax = plt.subplot(2, 2, 3)
    ax.plot(data[:,2])
    ax = plt.subplot(2, 2, 4)
    ax.plot(data[:,3])
    plt.show()

# Plot 3x2
if(n_plots == 6):
    plt.figure()
    ax = plt.subplot(2, 3, 1)
    ax.plot(data[:,0])
    ax = plt.subplot(2, 3, 2)
    ax.plot(data[:,1])
    ax = plt.subplot(2, 3, 3)
    ax.plot(data[:,2])
    ax = plt.subplot(2, 3, 4)
    ax.plot(data[:,3])
    ax = plt.subplot(2, 3, 5)
    ax.plot(data[:,4])
    ax = plt.subplot(2, 3, 6)
    ax.plot(data[:,5])
    plt.show()
