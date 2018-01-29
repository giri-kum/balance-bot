import numpy as np
import matplotlib.pyplot as plt
#TODO: this is just here as an example how to plot data from the motors for
# creating your motor model.  modify as you see fit.
alpha, theta, L_Enc, R_Enc, L_u, R_u, X, Y, bot_theta, error, in_P, out_P, in_I, out_I, in_D, out_D, x_dot, desired_alpha = np.loadtxt('pid.dat', delimiter=',', unpack=True)
imu, in_pid_d, out_pid_d = np.loadtxt('imu.dat', delimiter=',', unpack=True)
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
plt.plot(out_P,'r.-',label = 'out_P')
plt.legend(loc='lower left')
plt.subplot(426)
plt.plot(out_I,'r.-',label = 'out_I')
plt.legend(loc='lower left')
plt.subplot(428)
plt.plot(out_D,'r.-',label = 'out_D')
plt.legend(loc='lower left')	
plt.show()







