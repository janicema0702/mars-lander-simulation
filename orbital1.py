#straight down descent
import numpy as np
import matplotlib.pyplot as plt
G_constant = 6.674 * (10 ** -11)
M_constant = 6.42 * (10**23)

# simulation time, timestep and time
pt_max = 1 #1000
pdt = 0.1 #1
pt_array = np.arange(0, pt_max, pdt)

# (0,0,0) as center of the planet and 0 initial velocity
r_array = np.array([0, 0, 200000000])
v_array = np.array([0, 0, 0])

def planeteuler_int(r_array,v_array,pdt,pt_array):
    r_array_list = []
    v_array_list = []

    for t in pt_array:
        # append current state to trajectories
        r_array_list.append(r_array)
        v_array_list.append(v_array)

        # calculate new position and velocity
        r_array = r_array + np.dot(pdt, v_array)
        a_gravitational = -1* G_constant * M_constant * r_array[2] / ((np.linalg.norm(r_array[2]))**3)
        v_array = v_array + np.dot(pdt, a_gravitational)
        
    
    # convert trajectory lists into arrays
    r_trajectory_array = np.array(r_array_list)
    v_trajectory_array = np.array(v_array_list)

    #return (r_trajectory_array)
    #altitude
    altitude_list = [i[-1] for i in r_trajectory_array]

    # plot the graph (altitude as function of time)
    plt.figure(1)
    plt.clf()
    plt.xlabel('time (s)')
    plt.grid()
    plt.plot(pt_array, altitude_list, label='atitude (m)')
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
        a_gravitational = -1* G_constant * M_constant * r_array[2] / ((np.linalg.norm(r_array[2]))**3)
        v_array = np.dot(1/pdt, r_array-r_array_list[-2])
        r_array = 2* r_array - r_array_list[-2] + np.dot(pdt**2, a_gravitational)
        
        
    
    # convert trajectory lists into arrays
    r_trajectory_array = np.array(r_array_list[1:])
    v_trajectory_array = np.array(v_array_list)

    #return (r_trajectory_array)
    #altitude
    altitude_list = [i[-1] for i in r_trajectory_array]

    # plot the graph (altitude as function of time)
    plt.figure(1)
    plt.clf()
    plt.xlabel('time (s)')
    plt.grid()
    plt.plot(pt_array, altitude_list, label='atitude (m)')
    plt.legend()
    plt.show()

print(planeteuler_int(r_array,v_array,pdt,pt_array))
print(planetverlet_int(r_array,v_array,pdt,pt_array))