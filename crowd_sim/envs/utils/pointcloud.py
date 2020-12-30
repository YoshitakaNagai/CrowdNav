from crowd_sim.envs.utils.raycast import LiDAR
import math

class PointXY(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y

class PointCloudXY(object):
    def __init__(self):
        self.points = [None]

