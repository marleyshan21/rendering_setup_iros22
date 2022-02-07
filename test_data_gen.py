import numpy as np
import open3d as o3d
import cv2
import copy, os
import random



folder = "/root/ocrtoc_ws/src/rendering_setup_iros22/stapler_1_14929"



projection_img_file  =  os.path.join(folder, "projection.png") 
mask_img_file = os.path.join(folder, "mask.npy") 
pointcloud_file = os.path.join(folder, "pointcloud.npz") 
rotation_file = os.path.join(folder, "cls.txt") 
mask_img_array = np.load(mask_img_file)



#pointcloud check
pcd = np.load(pointcloud_file, allow_pickle=True)['data']
# print(pcd[0, :])
count  = 0
for point in pcd:
    
    print(point)
    if point[3] == 1:
        count = count + 1
print(count)
xyz = pcd[:, :3]
colors = pcd[:, 4:]
# print(colors.)
mask = pcd[:, 3]
# a_file = open("/root/ocrtoc_ws/src/rendering_setup_iros22/test.txt", "w")
# for row in pcd[:, :]:
#     np.savetxt(a_file, row)
# a_file.close()

pcd_check = True
if pcd_check:
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    o3d.io.write_point_cloud("/root/ocrtoc_ws/src/rendering_setup_iros22/data_gen_check.pcd", pcd)
    pcd_load = o3d.io.read_point_cloud("/root/ocrtoc_ws/src/rendering_setup_iros22/data_gen_check.pcd")
    o3d.visualization.draw_geometries([pcd_load])

#rotation file check 
original_mat = np.loadtxt(rotation_file).reshape(3, 3)
print(original_mat)


#img_check 

img = cv2.imread(projection_img_file)
cv2.imshow('projection', img)
cv2.waitKey()
#mask check
cv2.imshow("mask", mask_img_array )
cv2.waitKey()