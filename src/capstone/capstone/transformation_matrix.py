import numpy
from math import sin, cos

class Transformation:
    def __init__(self):
        self.matrix = numpy.identity(4)

    @property
    def rotation(self):
        return self.matrix[0:3,0:3]

    @rotation.setter
    def rotation(self, angles: list):
        if len(angles) != 3 or len(angles) != 4:
            raise ValueError(f"Setting rotation angle takes in a list of length 3 or 4, list of length {len(angles)} was given")

        if len(angles) == 3:
            roll, pitch, yaw = angles

            self.matrix[0][0] = cos(yaw) * cos(pitch)
            self.matrix[0][1] = -sin(yaw) * cos(roll) + cos(yaw) * sin(pitch) * sin(roll)
            self.matrix[0][2] = sin(yaw) * sin(roll) + cos(yaw) * sin(pitch) * cos(roll)
            self.matrix[1][0] = sin(yaw) * cos(pitch)
            self.matrix[1][1] = cos(yaw) * cos(roll) + sin(yaw) * sin(pitch) * sin(roll)
            self.matrix[1][2] = -cos(yaw) * sin(roll) + sin(yaw) * sin(pitch) * cos(roll)
            self.matrix[2][0] = -sin(pitch)
            self.matrix[2][1] = cos(pitch) * sin(roll)
            self.matrix[2][2] = cos(pitch) * cos(roll)

        elif len(angles) == 4:
            q1, q2, q3, q0 = angles

            self.matrix[0][0] = 2 * (q0 * q0 + q1 * q1) - 1
            self.matrix[0][1] = 2 * (q1 * q2 - q0 * q3)
            self.matrix[0][2] = 2 * (q1 * q3 + q0 * q2)
            self.matrix[1][0] = 2 * (q1 * q2 + q0 * q3)
            self.matrix[1][1] = 2 * (q0 * q0 + q2 * q2) - 1
            self.matrix[1][2] = 2 * (q2 * q3 - q0 * q1)
            self.matrix[2][0] = 2 * (q1 * q3 - q0 * q2)
            self.matrix[2][1] = 2 * (q2 * q3 + q0 * q1)
            self.matrix[2][2] = 2 * (q0 * q0 + q3 * q3) - 1

    @property
    def translation(self):
        return self.matrix[0:3,3:].flatten()

    @translation.setter
    def translation(self, new_translation):
        if len(new_translation) != 3:
            raise ValueError(f"Setting translation takes in a list of length 3, list of length {len(new_translation)} was given")

        self.matrix[0][3] = new_translation[0]
        self.matrix[1][3] = new_translation[1]
        self.matrix[2][3] = new_translation[2]

    def __mul__(self, other):
        if type(other) != Transformation:
            raise ValueError(f"Can't multiply Transforamtion and {type(other)}")

        new = Transformation()
        new.matrix = numpy.matmul(self.matrix, other.matrix)
        return new

    def apply(self, vector):
        if len(vector) != 3 or len(vector) != 4:
            raise ValueError(f"Can't apply transforamtion to vector that is not length 3 or 4, given {len(vector)}")

        if len(vector) == 3:
            if vector.shape != (3,):
                raise ValueError(f"Ensure the shape of the vector is either (3,) or (4,), given {vector.shape}")
            vector.append(1)

        if vector.shape != (4,):
            raise ValueError(f"Ensure the shape of the vector is either (3,) or (4,), given {vector.shape}")

        return matmul(self.matrix, vector)

    def apply_multiple(self, array):
        points = []
        for vector in array:
            point = self.apply(vector)
            points.append(point)
        return points