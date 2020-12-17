import sys
import numpy as np
import matplotlib.pyplot as plt

# USAGE
# python3 PlotStuff.py [folder on desktop] [data]
# python3 PlotStuff.py RobotData theta

if(len(sys.argv) <= 2):
    sys.exit()

if(sys.argv[2] == 'theta'):
    n_plots = 6
elif(sys.argv[2] == 'state'):
    n_plots = 4
elif(sys.argv[2] == 'ctrl'):
    n_plots = 1
elif(sys.argv[2] == 'K'):
    n_plots = 4
else:
    sys.exit()

file = open('/home/pi/Desktop/' + sys.argv[1] + '/RLS_' + sys.argv[2] + '.csv', 'r')

# file = open('/home/pi/Desktop/RobotData/RLS_theta.csv', 'r')
# file = open('/home/pi/Desktop/RobotData/RLS_state.csv', 'r')
# file = open('/home/pi/Desktop/RobotData/RLS_ctrl.csv', 'r')
# file = open('/home/pi/Desktop/RobotData/RLS_K.csv', 'r')

data = np.genfromtxt(file, delimiter=',')

# Plot 1x1
if(n_plots == 1):
    plt.figure()
    plt.plot(data)
    plt.show()

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
