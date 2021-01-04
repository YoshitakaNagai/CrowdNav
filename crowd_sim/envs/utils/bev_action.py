from collections import namedtuple
import math

ActionXY = namedtuple('ActionXY', ['vx', 'vy'])
ActionRot = namedtuple('ActionRot', ['v', 'r'])

class ActionConverter(object):
    def __init__(self):

    def to_vx_vy(self, linear_v, angular_v, yaw, dt):
        vx = linear_v * math.cos(yaw + angular_v * dt)
        vy = linear_v * math.sin(yaw + angular_v * dt)
        return vx, vy

    def to_linear_angular_v(self, vx, vy, dyaw, dt):
        linear_v = math.sqrt(vx * vx + vy * vy)
        angular_v = dyaw / dtself.robot
        return linear_v, angular_v
        
