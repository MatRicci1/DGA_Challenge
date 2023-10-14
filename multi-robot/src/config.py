#!/usr/bin/env python
import numpy as np
# Define the kill zones !!
# kill_zone = [x0, x1, y0, y1]
# x0 the lowest x of the zone, x1 the highest x of the zone
# y0 the lowest y of the zone, y1 the highest y of the zone
# For now we assume the kill zone spans everywhere in z

kill_zone_1 = {'x0': -40, 'x1': -24, 'y0': 23, 'y1': 53}
kill_zone_2 = {'x0': 33, 'x1': 142, 'y0': -30, 'y1': 10}

drone_angles = {"wfov": np.pi/2 * 16/12.0, "hfov": np.pi/2}
