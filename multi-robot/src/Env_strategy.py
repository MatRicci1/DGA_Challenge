#!/usr/bin/env python
import numpy as np
import math

########################################################################################
# Defines the main strategy executed by the drone in the environment
# Returns the position goals for each drone accordingly to the main strategy
########################################################################################

### Return all drones trajectory
def drone_strategy(n, r, h, pos, Env):
    ### Divide environment into n Zones
    D = r*2
    div = (Env['x1'] - Env['x0'])/n
    go_back = div/D
    go_back = math.ceil(go_back)
    step = div/(go_back*2)
    
    drones_traj_true = []
    drones_traj_goal = []

    ### Define all drones trajectory
    for i in range(n):
        ini = (step + i*div) + (Env['x0'] - pos[i][0])
        traj_true, traj_goal = drone_route(pos[i], Env, ini, go_back, step, h)
        drones_traj_true.append(np.array([0,0,0]))
        drones_traj_goal.append(traj_goal) 
    
    return drones_traj_true, drones_traj_goal

### Define single drone trajectory
def drone_route(drone_pos, Env, ini, go_back, step, h):

    route = [[0,0,h],
            [ini ,Env['y0']- drone_pos[1],0], 
            [0,Env['y1']-Env['y0'],0],
            ]

    for j in range(int(go_back)-1):
        
        a = 1
        if j%2 == 0:
            a = -1

        route.append([step*2,0,0])
        route.append([0,a*(Env['y1']-Env['y0']),0])
    
    traj_true = []
    traj_goal = []

    for i in range(len(route)):

        traj_true.append(drone_pos)
        new_pos = drone_pos + route[i]
        traj_goal.append(new_pos)
        drone_pos = new_pos

    return traj_true, traj_goal


### Define Strategy
n = 4                                                       # Number of drones for strategy 
Env = {'x0': -45.0, 'x1': 35.0, 'y0': -40.0, 'y1': 40.0}    # Environment boundaries
r = 8.0                                                     # Radious of field of view
h = 9.0                                                     # Height of drones

### Drones positions
drone1_pos = np.array([-32.0, -32.0, 0.0])
drone2_pos = np.array([-31.0, -32.0, 0.0])
drone3_pos = np.array([-30.0, -32.0, 0.0])
drone4_pos = np.array([-29.0, -32.0, 0.0])
drone5_pos = np.array([-28.0, -32.0, 0.0])

drone6_pos = np.array([-32.0, -34.0, 0.0])
drone7_pos = np.array([-31.0, -34.0, 0.0])
drone8_pos = np.array([-30.0, -34.0, 0.0])
drone9_pos = np.array([-29.0, -34.0, 0.0])
drone10_pos = np.array([-28.0, -34.0, 0.0])

drones_pos = [drone1_pos, drone2_pos, drone3_pos, drone4_pos, drone5_pos, drone6_pos, drone7_pos, drone8_pos, drone9_pos, drone10_pos]

drones_traj_true, drones_traj_goal = drone_strategy(n, r, h, drones_pos, Env)
