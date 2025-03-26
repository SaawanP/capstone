import h5py
import open3d as o3d
import cv2
import numpy as np
from sklearn.decomposition import PCA
from scipy.optimize import least_squares
from numpy.linalg import norm
import matplotlib.pyplot as plt
import math

from transformation_matrix import Transformation

# Fill in point cloud
def create_cylinder(radius, length, radius_slices, length_slices, transformation):
    pcd = o3d.geometry.PointCloud()
    points = np.zeros(shape=(radius_slices * length_slices, 3))

    theta_step_size = 2 * math.pi / radius_slices
    l_step_size = length / length_slices

    ind = 0
    for i in range(length_slices):
        z = i * l_step_size
        for j in range(radius_slices):
            x = radius * math.cos(j * theta_step_size)
            y = radius * math.sin(j * theta_step_size)
            points[ind] = [x, y, z]
            ind += 1

    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.rotate(transformation.rotation, center=(0,0,0))
    pcd.translate(transformation.translation + np.array([0,0,70]))
    return pcd

radius = 101.6
length = 1524
tranform = Transformation()
filled_pcd = create_cylinder(radius, length, 100, 500, tranform)
avg_color = [0.5, 0.5, 0.5]
filled_pcd.paint_uniform_color(avg_color)
colors = np.array(filled_pcd.colors)
points = np.array(filled_pcd.points)

cps = [points[4035], points[9066], points[15018], points[24997], points[36030]]
for cp in cps:
    for i, point in enumerate(points):
        if norm(cp - point) < 20:
            colors[i] = [255, 213, 0]

def key_callback(vis, action, mods):
    global isRunning
    if action == 0:
        isRunning = False

filled_pcd.colors = o3d.utility.Vector3dVector(colors)
isRunning = True
vis = o3d.visualization.VisualizerWithKeyCallback()
vis.create_window()
vis.register_key_action_callback(81, key_callback)
coordinateFrame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=[0, 0, 0])
vis.add_geometry(coordinateFrame)
vis.add_geometry(filled_pcd)

while isRunning:
    vis.poll_events()
    vis.update_renderer()

vis.destroy_window()