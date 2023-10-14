#!/usr/bin/env python
import numpy as np
import math

def rel_pos(pos, Env):
    new_pos  = pos + np.array([-Env['x0'],-Env['y0'], 0])
    return new_pos

def drone_strategy(n, r, h, pos, Env):

    Env_rel = {'x0': 0,
            'x1': -Env['x0'] + Env['x1'],
            'y0': 0,
            'y1': -Env['y0'] + Env['y1']}

    D = r*2
    div = Env_rel['x1']/n
    go_back = div/D
    go_back = math.ceil(go_back)
    step = div/(go_back*2)
    
    drones_traj_true = []
    drones_traj_goal = []

    for i in range(n):
        drone_rel_pos = rel_pos(pos[i], Env)
        ini = (step + i*div)
        traj_true, traj_goal = drone_route(drone_rel_pos, Env_rel, ini, go_back, step, h)
        drones_traj_true.append(traj_true)
        drones_traj_goal.append(traj_goal)

    return drones_traj_true, drones_traj_goal



def drone_route(drone_pos_rel, Env_rel, ini, go_back, step, h):

    route = [[0,0,h],
            [ini,0,0] - drone_pos_rel, 
            [0,Env_rel['y1'],0],
            ]

    for j in range(int(go_back)-1):
        
        a = 1
        if j%2 == 0:
            a = -1

        route.append([step*2,0,0])
        route.append([0,a*Env_rel['y1'],0])
    
    traj_true = []
    traj_goal = []

    for i in range(len(route)):

        traj_true.append(drone_pos_rel)
        new_pos = drone_pos_rel + route[i]
        traj_goal.append(new_pos)
        drone_pos_rel = new_pos

    return traj_true, traj_goal

# Define Environment !!
n = 1
Env = {'x0': -32, 'x1': 32, 'y0': -41, 'y1': 41}
r = 8
h = 10

drone1_pos = np.array([-32, -32, 0])
drone2_pos = np.array([-31, -32, 0])
drone3_pos = np.array([-30, -32, 0])
drone4_pos = np.array([-29, -32, 0])
drones_pos = [drone1_pos, drone2_pos, drone3_pos, drone4_pos]

drone1_gps = np.array([49.8600703049, 8.68748031965, -3.51534085388])
drone2_gps = np.array([49.8599734962, 8.6875684932, -2.07130799978])
drone3_gps = np.array([49.8599827901, 8.68750451018, -5.18067049565])
drone4_gps = np.array([49.8599920156, 8.68761834619, 5.18340736275])
drones_gps = [drone1_gps, drone2_gps, drone3_gps, drone4_gps]

drones_traj_true, drones_traj_goal = drone_strategy(n, r, h, drones_pos, Env)

# print(drones_traj_true)
# print(drones_traj_goal)