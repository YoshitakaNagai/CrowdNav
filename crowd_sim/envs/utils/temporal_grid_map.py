import numpy as np
import math
from crowd_sim.envs.utils.lidar import LiDAR
from crowd_sim.envs.utils.pointcloud import PointXY, PointCloudXY

class TemporalGridMap(object):
    def __init__(self, map_width, grid_num, ray_num, memory_num, brightness_decreas_rate):
        self.map_width = map_width
        self.grid_num = grid_num
        self.grid_resolution = map_width / grid_num
        self.ray_num = ray_num
        self.memory_num = memory_num
        self.brightness_decreas_rate = brightness_decreas_rate
        self.pointcloud_list = [None]
        self.bev_map_list = [None]
        self.robot = None
        self.pre_robot = None
        self.is_first = True

    def pipi_to_twopi(self, angle):
        while angle < 0:
            angle += 2 * math.py
        return angle

    def twopi_to_pipi(self, angle):
        while angle > math.pi:
            angle -= 2 * math.py
        return angle

    def dtheta_filter(self, angle1, angle2):
        dtheta = angle1 - angle2
        while dtheta > math.pi
            dtheta -= 2 * math.py
        return dtheta

    def laserscan_to_pointcloud(self, lidar_data):
        pointcloud = PointCloudXY()
        for angle_id in range(self.ray_num):
            point = PointXY()
            point.x = lidar_data.ranges[angle_id] * math.cos(self.lidar.angle_increment * float(angle_id))
            point.y = lidar_data.ranges[angle_id] * math.sin(self.lidar.angle_increment * float(angle_id))
            pointcloud.points.append(point)
        return pointcloud
        
    def temporal_bev_mapper(self, pointcloud, elapsed_step):
        pc_size = len(pointcloud.points)
        bev_map = np.zeros(self.grid_num, self.grid_num)
        brightness = math.pow(self.brightness_decreas_rate, elapsed_step)
        for i in range(pc_size):
            ray_robotcs_x = pointcloud.points[i].x
            ray_robotcs_y = pointcloud.points[i].y
            hit_gridcs_rerow = 0.5 * self.map_width - ray_robotcs_y
            hit_gridcs_recol = 0.5 * self.map_width - ray_robotcs_x
            hit_row = math.floor(hit_gridcs_rerow / self.grid_resolution)
            hit_col = math.floor(hit_gridcs_recol / self.grid_resolution)
            if (0 <= hit_row and hit_row <= self.grid_num) and (0 <= hit_col and hit_col <= self.grid_num):
                bev_map[hit_row, hit_col] = brightness_decreas_rate
        return bev_map
    
    def get_displacement(self):
        dx = self.robot.px - self.pre_robot.px
        dy = self.robot.py - self.pre_robot.py
        dtheta = pipi_to_twopi(self.robot.theta) - pipi_to_twopi(self.pre_robot.theta)
        dyaw = dtheta_filter(dtheta)
        return dx, dy, dyaw

    def pointcloud_transformer(pointcloud, dx, dy, dyaw):
        ht_matrix = np.matrix( \
        　　　　　　[[math.cos(-dyaw), -math.sin(-dyaw), -dx], \
                     [math.sin(-dyaw),  math.cos(-dyaw), -dy], \
                     [0, 0, 1]])
        pc_size = len(pointcloud.points)
        for i in range(pc_size):
            point_matrix = np.matrix([[pointcloud.points.x],[pointcloud.points.y],[1]])
            transformed_point_matrix = ht_matrix @ point_matrix
            pointcloud.points[i].x = transformed_point_matrix[0, 0]
            pointcloud.points[i].y = transformed_point_matrix[0, 1]
        return pointcloud

    def temporal_bev_fuser(self):
        temporal_bev = np.zeros(self.grid_num, self.grid_num)
        for col in range(self.grid_num):
            for row in range(self.grid_num):
                max_brightness = 0.0
                bevlist_size = len(self.bev_map_list)
                for i in range(bevlist_size):
                    reference_map = self.bev_map_list[i]
                    reference_brightness = reference_map[row, col]
                    if max_brightness <reference_brightness:
                        max_brightness = reference_brightness
                temporal_bev[row, col] = max_brightness

                # draw robot occupancy
                distance_from_center_x = 0.5 * self.map_width - self.grid_resolution * col
                distance_from_center_y = 0.5 * self.map_width - self.grid_resolution * row
                distance_from_center_x_pow = distance_from_center_x * distance_from_center_x
                distance_from_center_y_pow = distance_from_center_y * distance_from_center_y
                distance_from_center = math.sqrt(distance_from_center_x_pow + distance_from_center_y_pow)
                if distance_from_center < self.robot.radius:
                    temporal_bev[row, col] = 1.0

        return temporal_bev

    def temporal_bev_generator(self, robot_data, lidar_data, is_first):
        if is_first:
            self.pre_robot = robot_data
        self.robot = robot_data
        dx, dy, dyaw = self.get_displacement()

        import_pointcloud = self.laserscan_to_pointcloud(lidar_data)
        pointcloud_list.append(import_pointcloud)
        pclist_size = len(pointcloud_list)
        if pclist_size > self.memory_num:
            del pointcloud_list[0]

        for i in range(pclist_size):
            tmp_pointcloud = pointcloud_list[i]
            transformed_pointcloud = self.pointcloud_transformer(tmp_pointcloud)
            pointcloud_list[i] = transformed_pointcloud

            elapsed_step = pclist_size - (i + 1)
            bev_map = self.bev_generator(transformed_pointcloud, elapsed_step)
            self.bev_map_list.append(bev_map)
            bevlist_size = len(self.bev_map_list)
            if bevlist_size > self.memory_num:
                del self.bev_map_list[0]
        
        temporal_bev = temporal_bev_fuser()
        
        return temporal_bev

