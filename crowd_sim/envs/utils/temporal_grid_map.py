import numpy as np
import math
from crowd_sim.envs.utils.raycast import LiDAR

class TemporalGridMap(object):
    def __init__(self, map_width, lidar):
        self.map_width = map_width
        self.grid_num = bev_map.shape[0]
        self.grid_resolution = map_width / self.grid_num
        self.raycast_map = np.zeros((grid_num, grid_num))
        self.lidar = lidar
        self.ray_num = len(lidar)

    def bev_generator(self):
        for angle_id in range(self.ray_num):
            ray_robotcs_x = self.lidar.ranges[angle_id] * math.cos(self.lidar.angle_increment * float(angle_id))
            ray_robotcs_y = self.lidar.ranges[angle_id] * math.sin(self.lidar.angle_increment * float(angle_id))
            hit_gridcs_rerow = 0.5 * self.map_width - ray_robotcs_y
            hit_gridcs_recol = 0.5 * self.map_width - ray_robotcs_x
            hit_row = math.floor(hit_gridcs_rerow / self.grid_resolution)
            hit_col = math.floor(hit_gridcs_recol / self.grid_resolution)
            if (0 <= hit_row and hit_row <= self.grid_num) and (0 <= hit_col and hit_col <= self.grid_num):
                self.raycast_map[hit_row, hit_col] = 1.0
        
        
