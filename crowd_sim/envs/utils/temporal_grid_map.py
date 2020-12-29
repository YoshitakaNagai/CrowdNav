import numpy as np
from crowd_sim.envs.utils.raycast import RayCast

class TemporalGridMap(object):
    def __init__(self, map_width, grid_num):
        self.map_width = map_width
        self.grid_num = grid_num
        self.grid_resolution = map_width / grid_num

    def 
