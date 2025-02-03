
import numpy as np
import matplotlib.pyplot as plt
results = np.loadtxt("C:\\Users\\janic\\source\\repos\\lander\\lander\\ap.txt")


plt.figure(1)
plt.clf()
plt.xlabel('altitude (m)')
plt.ylabel('descent rate (m/s)')
plt.grid()
plt.plot(results[:, 0], results[:, 1], label = 'theoretical (m/s)')
plt.plot(results[:, 0], results[:, 2], label = 'experimental (m/s)')
plt.legend()
plt.show()