class LiDAR(object):
    def __init__(self, ray_num, horizontal_fov):
        self.angle_increment = horizontal_fov / float(ray_num)
        self.ranges = [100.0] * ray_num
