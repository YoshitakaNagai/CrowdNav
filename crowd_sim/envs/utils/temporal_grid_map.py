import numpy as np
import math
from crowd_sim.envs.utils.raycast import LiDAR
from crowd_sim.envs.utils.pointcloud import PointXY, PointCloudXY

class TemporalGridMap(object):
    def __init__(self, map_width, lidar):
        self.map_width = map_width
        self.grid_num = bev_map.shape[0]
        self.grid_resolution = map_width / self.grid_num
        self.raycast_map = np.zeros((grid_num, grid_num))
        self.lidar = lidar
        self.ray_num = len(lidar)
        self.pointcloud = PointCloudXY()
        self.pointcloud_list = [None]

    def scan_to_pointcloud(self):
        for angle_id in range(self.ray_num):
            point = PointXY()
            point.x = self.lidar.ranges[angle_id] * math.cos(self.lidar.angle_increment * float(angle_id))
            point.y = self.lidar.ranges[angle_id] * math.sin(self.lidar.angle_increment * float(angle_id))
            self.pointcloud.points.append(point)
        
    def bev_generator(self):
        pc_size = len(self.pointcloud.points)
        for i in range(pc_size):
            ray_robotcs_x = self.pointcloud.points[i].x
            ray_robotcs_y = self.pointcloud.points[i].y
            hit_gridcs_rerow = 0.5 * self.map_width - ray_robotcs_y
            hit_gridcs_recol = 0.5 * self.map_width - ray_robotcs_x
            hit_row = math.floor(hit_gridcs_rerow / self.grid_resolution)
            hit_col = math.floor(hit_gridcs_recol / self.grid_resolution)
            if (0 <= hit_row and hit_row <= self.grid_num) and (0 <= hit_col and hit_col <= self.grid_num):
                self.raycast_map[hit_row, hit_col] = 1.0
        
        
