import numpy as np
import matplotlib.pyplot as plt
#TODO: this is just here as an example how to plot data from the motors for
# creating your motor model.  modify as you see fit.
alpha, theta, L_Enc, R_Enc, L_u, R_u, X, Y, bot_theta, error, L_P, R_P, L_I, R_I, L_D, R_D, x_dot, desired_alpha = np.loadtxt('pid.dat', delimiter=',', unpack=True)
imu = np.loadtxt('imu.dat', delimiter=',', unpack=True)
plt.figure(1)
plt.subplot(421)
plt.plot(alpha,'r.-',label = 'alpha')
plt.plot(desired_alpha,'g.-',label = 'desired_alpha')
plt.legend(loc='lower left')
plt.subplot(423)
plt.plot(error,'r.-',label = 'error')
plt.legend(loc='lower left')
plt.subplot(425)
plt.plot(x_dot,'r.-',label = 'x_dot')
plt.legend(loc='lower left')	
plt.subplot(427)
plt.plot(imu,'r.-',label = 'imu')
plt.legend(loc='lower left')	


plt.subplot(422)
plt.plot(alpha,'r.-',label = 'alpha')
plt.plot(desired_alpha,'g.-',label = 'desired_alpha')
plt.legend(loc='lower left')
plt.subplot(424)
plt.plot(L_P,'r.-',label = 'L_P')
plt.legend(loc='lower left')
plt.subplot(426)
plt.plot(L_I,'r.-',label = 'L_I')
plt.legend(loc='lower left')
plt.subplot(428)
plt.plot(L_D,'r.-',label = 'L_D')
plt.legend(loc='lower left')	
plt.show()







