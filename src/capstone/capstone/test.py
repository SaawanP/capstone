import h5py
import open3d as o3d
import cv2
import numpy as np
from sklearn.decomposition import PCA
from scipy.optimize import least_squares
from numpy.linalg import norm
import matplotlib.pyplot as plt
import math
import os

from transformation_matrix import Transformation

# Function to create cylinder (unchanged)
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

def is_yellow_point(point_color, tolerance=0.2):
    """Check if a point color is close to yellow."""
    yellow_color = np.array([1, 0.835, 0])
    return np.all(np.isclose(point_color, yellow_color, atol=tolerance))

# Dictionary to map yellow points to specific images
yellow_point_images = {
    3637: "C:/Users/varun/Downloads/blockage.png",    # Example image path for first point
    9066: "C:/Users/varun/Downloads/blockage2.png",   # Different image for second point
    14417: "C:/Users/varun/Downloads/crack.png",   # Different image for third point
    24396: "C:/Users/varun/Downloads/blowout.png",   # Different image for fourth point
    35430: "C:/Users/varun/Downloads/play-circle.png"    # Different image for fifth point
}
def on_pick(event):
    # Get the picked point indices
    picked_indices = event.ind
    
    print(f"Picked indices: {picked_indices}")  # Debug print
    
    # Convert point cloud colors to numpy array for checking
    pcd_colors = np.asarray(filled_pcd.colors)
    
    # Check if any of the picked points are yellow
    yellow_picks = [
        idx for idx in picked_indices 
        if is_yellow_point(pcd_colors[idx])
    ]
    
    print(f"Yellow picks: {yellow_picks}")  # Debug print
    
    if yellow_picks:
        # Get the first yellow point index
        yellow_point_index = yellow_picks[0]
        
        print(f"Yellow point index: {yellow_point_index}")  # Debug print
        
        # Check if this yellow point has an associated image
        if yellow_point_index in yellow_point_images:
            image_path = yellow_point_images[yellow_point_index]
            
            print(f"Image path: {image_path}")  # Debug print
            
            if os.path.exists(image_path):
                img = cv2.imread(image_path)
                
                if img is not None:
                    cv2.imshow('Selected Point Image', img)
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()
                else:
                    print(f"Failed to read image: {image_path}")
            else:
                print(f"Image not found: {image_path}")
# Create cylinder
radius = 101.6
length = 1524
tranform = Transformation()
filled_pcd = create_cylinder(radius, length, 100, 500, tranform)
avg_color = [0.5, 0.5, 0.5]
filled_pcd.paint_uniform_color(avg_color)
colors = np.array(filled_pcd.colors)
points = np.array(filled_pcd.points)

# Add yellow points
cps = [points[4035], points[9066], points[15018], points[24997], points[36030]]
for cp in cps:
    for i, point in enumerate(points):
        if norm(cp - point) < 20:
            colors[i] = [1, 0.835, 0]  # Adjusted to match Open3D color format

filled_pcd.colors = o3d.utility.Vector3dVector(colors)

# Visualization setup
vis = o3d.visualization.Visualizer()
vis.create_window()

# Add coordinate frame
coordinateFrame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=[0, 0, 0])
vis.add_geometry(coordinateFrame)
vis.add_geometry(filled_pcd)

# Create matplotlib figure with improved visualization
fig = plt.figure(figsize=(12, 8))  # Larger figure size
ax = fig.add_subplot(111, projection='3d')

# Add text at the top of the figure
fig.suptitle(f'Cylinder Point Cloud\nRadius: {radius} mm, Length: {length} mm', 
             fontsize=16, fontweight='bold')

# Scatter plot with improved scaling and color
scatter = ax.scatter(points[:, 0], points[:, 1], points[:, 2], c=colors, picker=5, alpha=0.7)

# Improve axis scaling and view
ax.set_box_aspect((np.ptp(points[:, 0]), np.ptp(points[:, 1]), np.ptp(points[:, 2])))  # Equal aspect ratio
ax.set_xlabel('X (mm)')
ax.set_ylabel('Y (mm)')
ax.set_zlabel('Z (mm)')

# Allow pan and rotate with mouse
plt.tight_layout()
fig.canvas.mpl_connect('pick_event', on_pick)

# Run visualization
vis.run()
vis.destroy_window()
plt.show()