import math
import numpy as np
from transformations import SE2

class ForwardKinematics:
    @staticmethod
    def fk(theta1, theta2):
        theta_out = theta1 + theta2
        point_arm_2_1 = SE2.transform([0.0, 0.0], 1, 0, theta2, rotate_first=True)
        point_arm_2_0 = SE2.transform(point_arm_2_1, 1, 0, theta1, rotate_first=True)
        return (point_arm_2_0[0], point_arm_2_0[1], theta_out)

class InverseKinematics:
    @staticmethod
    def ik(x, y):
        if x**2 + y**2 > 2**2 + 0.00000001:
            return ()
        if x == 0 and y == 0:
            return (math.pi, -math.pi), (-math.pi, math.pi)
        q2 = math.acos((x**2 + y**2 - 1 - 1) / 2)
        q1_offset = math.atan2(math.sin(q2), (1 + math.cos(q2)))
        q1_offset_alt = math.atan2(math.sin(-q2), (1 + math.cos(-q2)))
        q1 = math.atan2(y, x) - q1_offset
        q1_alt = math.atan2(y, x) - q1_offset_alt
        return (q1, q2), (q1_alt, -q2)