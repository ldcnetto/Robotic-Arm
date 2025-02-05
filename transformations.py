import numpy as np
import math

class SE2:
    @staticmethod
    def rectify_angle(theta):
        return theta % (2 * math.pi)

    @staticmethod
    def SE2_generic(x, y, theta):
        return np.matrix([[math.cos(theta), -math.sin(theta), x],
                          [math.sin(theta), math.cos(theta), y],
                          [0, 0, 1]])

    @staticmethod
    def SE2_xy(x, y):
        return SE2.SE2_generic(x, y, 0)

    @staticmethod
    def SE2_theta(theta):
        return SE2.SE2_generic(0, 0, theta)

    @staticmethod
    def transform(input, x, y, theta, rotate_first=False):
        a = np.concatenate((input, np.array([1])), axis=None)
        transform_matrix = np.matmul(SE2.SE2_xy(x, y), SE2.SE2_theta(theta))
        if rotate_first:
            transform_matrix = np.matmul(SE2.SE2_theta(theta), SE2.SE2_xy(x, y))
        output = np.array(np.matmul(transform_matrix, a)[0])[0]
        return output[:-1]