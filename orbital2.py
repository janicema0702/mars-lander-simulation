# Circular orbit

import numpy as np
import matplotlib.pyplot as plt
G_constant = 6.674 * 10e-11
M_constant = 6.42 * 10e23
r0 = 6*10e6
height = 0
e = 1 
#ellipse 0 < e < 1
# hyperbolic escape 1 < e

# simulation time, timestep and time
pt_max = 50000 #1000
pdt = 1 #1
pt_array = np.arange(0, pt_max, pdt)

# (0,0,0) as center of the planet and 0 initial velocity
r_array = np.array([r0 + height, 0, 0])
v_array = np.array([0, e*np.sqrt(G_constant*M_constant/(r0+height)), 0])

def planeteuler_int(r_array,v_array,pdt,pt_array):
    r_array_list = []
    v_array_list = []

    for t in pt_array:
        # append current state to trajectories
        r_array_list.append(r_array)
        v_array_list.append(v_array)

        # calculate new position and velocity
        r = r_array - np.array([0,0,0])
        a_gravitational = -1* G_constant * M_constant * r / ((np.linalg.norm(r))**3)

        r_array = r_array + np.dot(pdt, v_array)
        v_array = v_array + np.dot(pdt, a_gravitational)
        
        
        
    
    # convert trajectory lists into arrays
    orbital_trajectory_x = np.array([r_array_list[i][0] for i in range(len(r_array_list))])
    orbital_trajectory_y = np.array([r_array_list[i][1] for i in range(len(r_array_list))])


    # plot the graph (altitude as function of time)
    plt.title('EULER CIRCULAR ORBIT')
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.grid()
    plt.plot(orbital_trajectory_x, orbital_trajectory_y, label='trajectory')
    plt.legend()
    plt.show()

def planetverlet_int(r_array,v_array,pdt,pt_array):
    r_array_list = [r_array - np.dot(pdt, v_array)]
    v_array_list = []

    for t in pt_array:
        # append current state to trajectories
        r_array_list.append(r_array)
        v_array_list.append(v_array)

        # calculate new position and velocity
        r = r_array - np.array([0,0,0])
        a_gravitational = -1* G_constant * M_constant * r / ((np.linalg.norm(r))**3)

        v_array = np.dot(1/pdt, r_array-r_array_list[-2])
        r_array = 2* r_array - r_array_list[-2] + np.dot(pdt**2, a_gravitational)
        

    #convert trajectory lists into arrays
    orbital_trajectory_x = np.array([r_array_list[i][0] for i in range(len(r_array_list[1:]))])
    orbital_trajectory_y = np.array([r_array_list[i][1] for i in range(len(r_array_list[1:]))])

    # plot the graph (altitude as function of time)
    plt.title('VERLET CIRCULAR ORBIT')
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.grid()
    plt.plot(orbital_trajectory_x, orbital_trajectory_y, label='trajectory')
    plt.legend()
    plt.show()

print(planetverlet_int(r_array,v_array,pdt,pt_array))
#print(planetverlet_int(r_array,v_array,pdt,pt_array))
