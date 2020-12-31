import copy
import numpy as np
import math
from crowd_sim.envs.utils.human import Human
from crowd_sim.envs.utils.lidar import LiDAR
from crowd_sim.envs.utils.cell import Cell


class RayCast(object):
    def __init__(self, map_width, grid_num, ray_fov, ray_num, robot_data, ob_data, human_num, human_action_num):
        self.map_width = map_width
        self.grid_num = grid_num
        self.grid_resolution
        self.ray_num = ray_num
        self.ray_fov = ray_fov
        self.angle_resolution = ray_fov / ray_num
        self.precast_map_angle_id = np.zeros((grid_num, grid_num))
        self.precast_map_range = np.zeros((grid_num, grid_num))
        self.grid_map = np.zeros((grid_num, grid_num))
        self.ob_data = ob_data
        self.robot_data = robot_data
        self.human_data = [None]
        self.human_num = human_num
        self.human_action_num = human_action_num
        self.robotcs_human_data = [None]
        self.lidar = LiDAR(ray_num, ray_fov)
        self.human_distance_list = [None]
        self.robotcs_human_list = [None]

    def pipi_to_twopi(self, angle):
        while angle < 0:
            angle += 2 * math.py
        return angle

    def twopi_to_pipi(self, angle):
        while angle > math.pi:
            angle -= 2 * math.py
        return angle

    def get_angle_id(self, angle):
        angle = self.pipi_to_2pi(angle)
        return math.floor(angle / self.angle_resolution)
    
    def get_distance(x, y):
        return math.sqrt(x * x + y * y)

    def precast(self):
        for col in range(self.grid_num):
            for row in range(self.grid_num):
                relative_robotcs_x = 0.5 * self.map_width - (float(col) + 0.5) * self.grid_resolution
                relative_robotcs_y = 0.5 * self.map_width - (float(row) + 0.5) * self.grid_resolution
                relative_robotcs_angle = math.atan2(relative_robotcs_y, relative_robotcs_x)
                self.precast_map_angle_id[row, col] = self.get_angle_id(relative_robotcs_angle)
                self.precast_map_range[row, col] = self.get_distance(relative_robotcs_x, relative_robotcs_y)

    def human_loader(self):
        self.human_data.clear()
        for human_id, action_id in zip(self.human_num, self.human_action_num):
            self.human_data.append(self.ob_data[human_id])

    def human_transformer(self):
        self.robotcs_human_data = copy.deepcopy(self.human_data)
        for human_id in range(len(self.human_data)):
            human_dxg = self.human_data[human_id].px - self.robot_data.px
            human_dyg = self.human_data[human_id].py - self.robot_data.py
            human_distance = self.get_distance(human_dxg, human_dyg)
            human_directiong = math.atan2(human_dy, human_dx)
            human_directionr = human_directiong + twopi_to_pipi(robot_data.theta)
            human_directionr = twopi_to_pipi(human_directionr)
            robotcs_human_data[human_id].px = human_distance * math.cos(human_directionr)
            robotcs_human_data[human_id].py = human_distance * math.sin(human_directionr)
            
    def grid_plotter(self):
        human_grid_list = [None]
        for human_id in range(len(self.robotcs_human_data)):
            human_robotcs_x = self.robotcs_human_data[human_id].px
            human_robotcs_y = self.robotcs_human_data[human_id].py
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
                    distance = self.get_distance(relative_rerow, relative_recol)
                    if distance < self.robotcs_human_data[target_human_id].radius:
                        self.grid_map[row, col] = 1.0

    def lidar_raycast(self):
        for col in range(self.grid_num):
            for row in range(self.grid_num):
                if self.grid_map > 0:
                    target_angle_id = self.precast_map_angle_id[row, col]
                    target_range = self.precast_map_range[row, col]
                    if target_range < self.lidar.ranges[target_angle_id]:
                        self.lidar.ranges[target_angle_id] = target_range

    
    def laserscan_callback(self, is_first):
        if is_first
            self.precast()
        self.human_loader()
        self.human_transformer()
        self.grid_plotter()
        self.lidar_raycast()
        return self.lidar
        
