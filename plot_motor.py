import numpy as np
import matplotlib.pyplot as plt
#TODO: this is just here as an example how to plot data from the motors for
# creating your motor model.  modify as you see fit.
t, l_w, r_w, l_c, r_c, f = np.loadtxt('balancebot-F18/motor.dat', delimiter=',', unpack=True)
plt.figure(1)
plt.subplot(311)
plt.plot(t,l_w)
plt.plot(t,r_w)
plt.subplot(312)
plt.plot(t,l_c)
plt.plot(t,r_c)
plt.subplot(313)
plt.plot(t,f)
plt.show()