import numpy as np
import open3d as o3d
import os

if __name__ == "__main__":

    print("Load a ply point cloud, print it, and render it")
    if os.path.exists("/root/ocrtoc_ws/src/rendering_setup_iros22/saved_models/orion_pie/0.800000.pcd"):
        print("True")
    pcd = o3d.io.read_point_cloud("/root/ocrtoc_ws/src/rendering_setup_iros22/saved_models/orion_pie/0.800000.pcd")
    # pcd = o3d.io.read_point_cloud("../../TestData/fragment.pcd")
    # print(pcd)
    # print(np.asarray(pcd.points))
    o3d.visualization.draw_geometries([pcd])