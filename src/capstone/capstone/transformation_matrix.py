import numpy as np
from numpy.linalg import norm, inv
import math
from scipy.spatial.transform import Rotation as R

class Transformation:
    def __init__(self, rotation=None, translation=None):
        self.matrix = np.identity(4)

        if rotation is not None:
            self.rotation = rotation
        if translation is not None:
            self.translation = translation

    def __str__(self):
        matrix = self.matrix.__str__()
        rotation = self.rotation.__str__()
        translation = self.translation.__str__()

        return f"The full matrix is: \n {matrix} \n The rotation matrix is: \n {rotation} \n The translation is: \n {translation} \n"

    @property
    def rotation(self):
        return self.matrix[0:3, 0:3]

    @rotation.setter
    def rotation(self, inp: list):
        th = 1e-10
        if inp.shape == (3, 3):
            for i in range(3):
                for j in range(3):
                    if inp[i,j] < th and inp[i,j] > -th:
                        inp[i,j] = 0
            self.matrix[0:3, 0:3] = inp
        else:
            raise ValueError("input array is the wrong size")

    def vector_to_vector(self, final_vector, starting_vector):
        final_vector = np.array(final_vector)
        starting_vector = np.array(starting_vector)

        axis = np.cross(final_vector, starting_vector)
        c = np.dot(-final_vector, starting_vector)
        theta = math.acos(c / (norm(final_vector) * norm(starting_vector)))

        quat = [0, 0, 0, 1]
        if not (axis.all(where=0) and theta == math.pi):
            mag = math.sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)
            axis /= mag
            axis = axis * math.sin(theta / 2)
            quat = [axis[0], axis[1], axis[2], math.cos(theta/2)]
        self.rotation = R.from_quat(quat).as_matrix()

    @property
    def translation(self):
        return self.matrix[0:3, 3:].flatten()

    @translation.setter
    def translation(self, new_translation):
        self.matrix[0][3] = new_translation[0]
        self.matrix[1][3] = new_translation[1]
        self.matrix[2][3] = new_translation[2]

    def __mul__(self, other):
        if type(other) != Transformation:
            raise ValueError(f"Can't multiply Transforamtion and {type(other)}")

        new = Transformation()
        new.matrix = np.matmul(self.matrix, other.matrix)
        return new

    def apply(self, vector):
        if len(vector) != 3 and len(vector) != 4:
            raise ValueError(f"Can't apply transforamtion to vector that is not length 3 or 4, given {len(vector)}")

        if len(vector) == 3:
            if vector.shape != (3,):
                raise ValueError(f"Ensure the shape of the vector is either (3,) or (4,), given {vector.shape}")
            vector = np.append(vector, 1)

        if vector.shape != (4,):
            raise ValueError(f"Ensure the shape of the vector is either (3,) or (4,), given {vector.shape}")

        return np.matmul(self.matrix, vector)

    def apply_multiple(self, array):
        array = np.transpose(array)
        ones = np.ones(shape=(1, array.shape[1]))
        array = np.append(array, ones, axis=0)
        a = np.matmul(self.matrix, array)[:3]
        return np.transpose(a)

    def invert(self):
        self.matrix = inv(self.matrix)
