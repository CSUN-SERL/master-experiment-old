import tf.transformations
import math
import numpy

def quaternion_to_euler_angle(x, y, z, w):
    quaternion = (x, y, z, w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    return euler[0], euler[1], euler[2]


def shift_points(RX, RY, HX, HY):
    HX = HX - RX
    HY = HY - RY
    return HX, HY


def cartesian_to_polar_distance(x, y):
    return math.sqrt(x ** 2 + y ** 2)


def cartesian_to_polar_angle(x, y):
    return math.atan2(y, x)


def normalize_to_angle(vecX, vecY, theta):
    vecArray = numpy.matrix([[float(vecX)], [float(vecY)]])
    theta *= -1.0
    thetaCos = math.cos(theta)
    thetaSin = math.sin(theta)
    rotationMatrix = numpy.matrix([[thetaCos, -1 * thetaSin], [thetaSin, thetaCos]])

    rotation = numpy.matmul(rotationMatrix, vecArray)
    return rotation[0, 0], rotation[1, 0]