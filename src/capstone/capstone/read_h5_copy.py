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


pcd = o3d.geometry.PointCloud()
with h5py.File("data_storage.h5", "r") as f:
    for frame in f.keys():
        print(f"Frame: {frame}")
        np_array = f[frame]["numpy_points"][:]
        np_color = f[frame]["colors"][:]
        pcd.points.extend(o3d.utility.Vector3dVector(np_array))
        pcd.colors.extend(o3d.utility.Vector3dVector(np_color))
        metadata_list = eval(f[frame].attrs["metadata_list"])  # Convert string back to list
        timestamp = f[frame].attrs["timestamp"]

def find_cylinder(pcd):
    def fit_func(p, x, y, z):
        xc, yc, zc, r, a, b, c = p
        norm_factor = norm([a, b, c])
        a, b, c = a / norm_factor, b / norm_factor, c / norm_factor

        # Calculate projection of points onto the axis
        x_shift, y_shift, z_shift = x-xc, y-yc, z-zc
        dot_product = a * x_shift + b * y_shift + c * z_shift
        x_proj, y_proj, z_proj = dot_product * a, dot_product * b, dot_product * c
        radial_distance = np.sqrt((x_shift - x_proj) ** 2 + (y_shift - y_proj) ** 2 + (z_shift - z_proj) ** 2)

        radius_residual = radial_distance - r
        return radius_residual

    points = np.array(pcd.points)
    x, y, z = points[:, 0], points[:, 1], points[:, 2]

    p_initial = [0, 0, 0, 10, 1, 1, 1]
    est_p = least_squares(fit_func, p_initial, args=(x, y, z), xtol=1e-9, max_nfev=100)
    xc, yc, zc, r, a, b, c = est_p.x
    axis = np.array([a, b, c])
    axis /= norm(axis)

    # calculate centre
    dot = np.matmul(points, axis)
    dist = max(dot) - min(dot)
    # center = axis * (min(dot) + dist/2)

    t = Transformation(translation=[xc, yc, zc])
    t.vector_to_vector(axis, [0, 0, -1])
    return t, r, dist


def filter_pcd(pcd, transform, radius, threshold):
    def point_dist(point):
        axis = np.matmul(transform.rotation, [0,0,1])
        point_shifted = point - transform.translation
        dot = np.dot(axis, point_shifted)
        point_proj = dot * axis
        dist = norm(point_shifted-point_proj)

        if dist > radius - threshold and dist < radius + threshold:
            return True
        return False

    points = np.asarray(pcd.points)
    mask = [point_dist(point) for point in points]
    pcd.points = o3d.utility.Vector3dVector(points[mask])
    color = np.asarray(pcd.colors)
    pcd.colors = o3d.utility.Vector3dVector(color[mask])
    return pcd

# print('Finding cylinder')
# tranform, radius, length = find_cylinder(pcd)
# print(length)
# print('Filter pcd')
# # pcd = filter_pcd(pcd, tranform, radius, radius/100)
# points = np.array(pcd.points)
# max = np.max(points, axis=0)
# min = np.min(points, axis=0)
# center = (max + min) / 2
# tranform.translation = center

colors = np.array(pcd.colors)
avg_color = np.average(colors, axis=0)
print(avg_color)
exit()
cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius, length)
cylinder.rotate(tranform.rotation, center=[0,0,0])
cylinder.translate(tranform.translation)

isRunning = True

def key_callback(vis, action, mods):
    global isRunning
    if action == 0:
        isRunning = False

vis = o3d.visualization.VisualizerWithKeyCallback()
vis.create_window()
vis.register_key_action_callback(81, key_callback)
coordinateFrame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=[0, 0, 0])
vis.add_geometry(coordinateFrame)
vis.add_geometry(pcd)
arrow = o3d.geometry.TriangleMesh.create_arrow(cylinder_height=length-100, cylinder_radius=5, cone_radius=8, cone_height=20)
arrow.rotate(tranform.rotation, center=[0,0,0])
arrow.translate(tranform.translation)
arrow.paint_uniform_color([1,0,0])
cylinder_line = o3d.geometry.LineSet.create_from_triangle_mesh(cylinder)
vis.add_geometry(arrow)

while isRunning:
    vis.poll_events()
    vis.update_renderer()

vis.destroy_window()

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
            if ind == 400:
                print([x, y, z])

    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.rotate(transformation.rotation, center=[0,0,0])
    pcd.translate(transformation.translation + np.array([0,0,70]))
    return pcd

filled_pcd = create_cylinder(radius, length, 100, 100, tranform)
filled_pcd.paint_uniform_color(avg_color)
colors = np.array(filled_pcd.colors)
points = np.array(filled_pcd.points)

cp = [115.63933999, -58.17077169, 949.88924624]
for i, point in enumerate(points):
    if norm(cp-point) < 20:
        colors[i] = [255, 213, 0]

cp = [39.93833525, -62.29755771, 727.15278495]
for i, point in enumerate(points):
    if norm(cp-point) < 20:
        colors[i] = [255, 213, 0]

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