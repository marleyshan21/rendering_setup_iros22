import open3d as o3d
import os

import numpy as np
import copy
import cv2

print("Convert mesh to a point cloud and estimate dimensions")
folder_path = "/root/ocrtoc_ws/src/rendering_setup_iros22/training_models/orion_pie"

mesh_file = os.path.join(folder_path, 'textured.obj')
mesh = o3d.io.read_triangle_mesh(mesh_file)
pcd = mesh.sample_points_poisson_disk(5000)
diameter = np.linalg.norm(
    np.asarray(pcd.get_max_bound()) - np.asarray(pcd.get_min_bound()))
o3d.visualization.draw_geometries([pcd])


print("Define parameters used for hidden_point_removal")
camera = [0.1, 0.15, 0]
radius = diameter * 100

print("Get all points that are visible from given view point")
_, pt_map = pcd.hidden_point_removal(camera, radius)

print("Visualize result")
pcd = pcd.select_by_index(pt_map)
o3d.visualization.draw_geometries([pcd])