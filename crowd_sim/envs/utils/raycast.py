import numpy as np
import math
from crowd_sim.envs.utils.human import Human

class Cell(object):
    def __init__(self, row, col, agent_id):
        self.row = row
        self.col = col
        self.agent_id = agent_id

class LiDAR(object):
    def __init__(self, ray_num, horizontal_fov):
        self.angle_increment = horizontal_fov / float(ray_num)
        self.ranges = [100.0] * ray_num

class RayCast(object):
    def __init__(self, map_width, grid_num, ray_fov, ray_num, robot_data):
        self.map_width = map_width
        self.grid_num = grid_num
        self.grid_resolution
        self.ray_num = ray_num
        self.ray_fov = ray_fov
        self.angle_resolution = ray_fov / ray_num
        self.precast_map_angle_id = np.zeros((grid_num, grid_num))
        self.precast_map_range = np.zeros((grid_num, grid_num))
        self.grid_map = np.zeros((grid_num, grid_num))
        self.raycast_map = np.zeros((grid_num, grid_num))
        self.robot_data = robot_data
        self.human_data = [None]
        self.lidar = LiDAR(ray_num, ray_fov)

    def get_angle_id(self, angle):
        if angle < 0:
            angle += 2 * math.py
        return math.floor(angle / self.angle_resolution)
    
    def get_distance(x, y):
        return math.sqrt(x * x + y * y)

    def precast(self):
        for col in range(self.grid_num):
            for row in range(self.grid_num):
                relative_robotcs_x = 0.5 * self.grid_resolution - (float(col) + 0.5) * grid_resolution
                relative_robotcs_y = 0.5 * self.grid_resolution - (float(row) + 0.5) * grid_resolution
                relative_robotcs_angle = math.atan2(relative_robotcs_y, relative_robotcs_x)
                self.precast_map_angle_id[row, col] = get_angle_id(relative_robotcs_angle)
                self.precast_map_range[row, col] = get_distance(relative_robotcs_x, relative_robotcs_y)

    def human_loader(self, ob_data, human_num, human_action_num):
        self.human_data.clear()
        for human_id, action_id in zip(human_num, human_action_num):
            self.human_data.append(ob_data[human_id])
            
    def human_transformer(self):
        for human_id in range(len(self.human_data)):
            self.human_data[human_id].px = self.human_data[human_id].px - self.robot_data.px
            self.human_data[human_id].py = self.human_data[human_id].py - self.robot_data.py

    def grid_plotter(self):
        human_grid_list = [None]
        for human_id in range(len(self.human_data)):
            human_robotcs_x = self.human_data[human_id].px
            human_robotcs_y = self.human_data[human_id].py
            on_gridcs_rerow = 0.5 * self.map_width - human_robotcs_y
            on_gridcs_recol = 0.5 * self.map_width - human_robotcs_x
            on_row = math.floor(on_gridcs_rerow / self.grid_resolution)
            on_col = math.floor(on_gridcs_recol / self.grid_resolution)
            if (0 <= on_row and on_row <= self.grid_num) and (0 <= on_col and on_col <= self.grid_num):
                # self.grid_map[on_row, on_col] = 1.0
                human_on_cell = Cell(on_row, on_col, human_id)
                human_grid_list.append(human_on_cell)

        for cell_id in range(len(human_grid_list)):
            target_cell = human_grid_list[cell_id]
            target_human_id = target_cell.agent_id
            for col in range(self.grid_num):
                for row in range(self.grid_num):
                    relative_rerow = self.grid_resolution * (row - target_cell.row)
                    relative_recol = self.grid_resolution * (row - target_cell.col)
                    distance = get_distance(relative_rerow, relative_recol)
                    if distance < self.human_data[target_human_id].radius:
                        self.grid_map[row, col] = 1.0

    def raycast(self):
        for col in range(self.grid_num):
            for row in range(self.grid_num):
                if self.grid_map > 0:
                    target_angle_id = self.precast_map_angle_id[row, col]
                    target_range = self.precast_map_range[row, col]
                    if target_range < self.lidar.ranges[target_angle_id]:
                        self.lidar.ranges[target_angle_id] = target_range

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
                    












    
