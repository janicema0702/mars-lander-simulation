#runtime - 3.763s
# uncomment the next line if running in a notebook
# %matplotlib inline
import numpy as np
import matplotlib.pyplot as plt

# mass, spring constant, initial position and velocity
m = 1
k = 1
x = 0
v = 1

# simulation time, timestep and time
t_max = 100 #1000
dt = 0.1 #1
t_array = np.arange(0, t_max, dt)


def euler_int(m,k,x,v,dt,t_array):
    x_list = []
    v_list = []

    for t in t_array:
        # append current state to trajectories
        x_list.append(x)
        v_list.append(v)

        # calculate new position and velocity
        a = -k * x / m
        x = x + dt * v
        v = v + dt * a
    
    # convert trajectory lists into arrays, so they can be sliced (useful for Assignment 2)
    x_array = np.array(x_list)
    v_array = np.array(v_list)

    # plot the position-time graph
    plt.figure(1)
    plt.clf()
    plt.xlabel('time (s)')
    plt.grid()
    plt.plot(t_array, x_array, label='x (m)')
    plt.plot(t_array, v_array, label='v (m/s)')
    plt.legend()
    plt.show()



def verlet_int(m,k,x,v,dt,t_array):
    x_list = [x-v*dt]
    v_list = []

    for t in t_array:
        # append current state to trajectories
        x_list.append(x)
        v_list.append(v)

        # calculate new position and velocity
        a = -k * x / m
        v = (1/dt) * (x - x_list[-2])
        x = 2*x -x_list[-2]+ a*(dt**2)
    
    # convert trajectory lists into arrays, so they can be sliced (useful for Assignment 2)
    x_array = np.array(x_list[1:])
    v_array = np.array(v_list)

    # plot the position-time graph
    plt.figure(1)
    plt.clf()
    plt.xlabel('time (s)')
    plt.grid()
    plt.plot(t_array, x_array, label='x (m)')
    plt.plot(t_array, v_array, label='v (m/s)')
    plt.legend()
    plt.show()

#print(euler_int(m,k,x,v,dt,t_array),verlet_int(m,k,x,v,dt,t_array))
#print(verlet_int(m,k,x,v,dt,t_array))
