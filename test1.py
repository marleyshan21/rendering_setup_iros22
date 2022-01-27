
import time
import pybullet as pb
physicsClient = pb.connect(pb.GUI)
import os
import pybullet_data
import open3d as o3d
import numpy as np
pb.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"))
pb.loadURDF("/root/ocrtoc_ws/src/pybullet_tests/table/table.urdf")

print(os.path.join(pybullet_data.getDataPath()))

visualShapeId = pb.createVisualShape(
    shapeType=pb.GEOM_MESH,
    fileName='/root/ocrtoc_ws/src/pybullet_tests/models/clear_box/textured.obj',
    rgbaColor=None,
    meshScale=[1, 1, 1])

collisionShapeId = pb.createCollisionShape(
    shapeType=pb.GEOM_MESH,
    fileName='/root/ocrtoc_ws/src/pybullet_tests/models/clear_box/collision.obj',
    meshScale=[1, 1, 1])

multiBodyId = pb.createMultiBody(
    baseMass=1.0,
    baseCollisionShapeIndex=collisionShapeId, 
    baseVisualShapeIndex=visualShapeId,
    basePosition=[0, 0, 1.0],
    baseOrientation=pb.getQuaternionFromEuler([0, 0, 0]))

pb.setGravity(0, 0, -9.8)
pb.setRealTimeSimulation(1)

viewMatrix = pb.computeViewMatrix(
    cameraEyePosition=[1.25, 0 ,1.75],
    cameraTargetPosition=[0, 0, 0.39],
    cameraUpVector=[-1, 0, 0])

projectionMatrix = pb.computeProjectionMatrixFOV(
    fov=65.0,
    aspect=1.0,
    nearVal=0.1,
    farVal=3.1)

# print("test!!!")
mesh = o3d.io.read_triangle_mesh("/root/ocrtoc_ws/src/pybullet_tests/models/clear_box/visual.ply")
# print("test2!!!")
pcd = mesh.sample_points_poisson_disk(2000, init_factor=5, pcl=None)
frame_init = o3d.geometry.TriangleMesh.create_coordinate_frame()
frame_init_r = copy.deepcopy(frame_init)
R = pcd.get_rotation_matrix_from_xyz((0, 0, 0.5 * np.pi))

print(R)
pcd = pcd.rotate(R, center=True)
mesh_r.rotate(mesh.get_rotation_matrix_from_xyz((np.pi / 2, 0, np.pi / 4)),
              center=(0, 0, 0))
rotation_mat = open("/root/ocrtoc_ws/src/pybullet_tests/saved_models/clear_box_rotation.txt", "w+")
for row in R:
    np.savetxt(rotation_mat, row)

rotation_mat.close()

# original_mat = np.loadtxt("/root/ocrtoc_ws/src/pybullet_tests/saved_models/clear_box_rotation.txt").reshape(3, 3)
# print(original_mat)

# o3d.visualization.draw_geometries([pcd])

o3d.io.write_point_cloud("/root/ocrtoc_ws/src/pybullet_tests/saved_models/clear_box.pcd", pcd)

# print("visual!!!!!!!!")
time.sleep(0.5)

width, height, rgbImg, depthImg, segImg = pb.getCameraImage(
    width=224, 
    height=224,
    viewMatrix=viewMatrix,
    projectionMatrix=projectionMatrix)

while 1:
    pb.stepSimulation()
    time.sleep(1./240)