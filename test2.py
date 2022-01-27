
import time
import pybullet as pb
physicsClient = pb.connect(pb.GUI)
import os
import copy
import cv2
import pybullet_data
import open3d as o3d
import numpy as np
import random

pb.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"))
pb.loadURDF("/root/ocrtoc_ws/src/rendering_setup_iros22/table/table.urdf")


directory = '/root/ocrtoc_ws/src/rendering_setup_iros22/training_models'
save_directory = '/root/ocrtoc_ws/src/rendering_setup_iros22/saved_models'

# def save_data():
def object_setup(visual_shape_file, collision_shape_file, texture_path, rotation):
    
    # print(rotation)
    
    visualShapeId = pb.createVisualShape(
            shapeType=pb.GEOM_MESH,
            fileName=visual_shape_file,
            rgbaColor=None,
            meshScale=[1, 1, 1])

    collisionShapeId = pb.createCollisionShape(
        shapeType=pb.GEOM_MESH,
        fileName=collision_shape_file,
        meshScale=[1, 1, 1])

    multiBodyId = pb.createMultiBody(
        baseMass=1.0,
        baseCollisionShapeIndex=collisionShapeId, 
        baseVisualShapeIndex=visualShapeId,
        basePosition=[0, 0, 0.9],
        baseOrientation=pb.getQuaternionFromEuler([0, 0, rotation]))
    
    
    textureId = pb.loadTexture(texture_path)
    pb.changeVisualShape(multiBodyId, -1, textureUniqueId=textureId)
    return multiBodyId

def delete_object(multiBodyId):
    
    pb.removeBody(multiBodyId)

def main():
    for root, subdirectories, files in os.walk(directory):
        for subdirectory in subdirectories:
            print(os.path.join(root, subdirectory))
            print(subdirectories)
            folder_path = os.path.join(root, subdirectory)
            visual_shape_file = os.path.join(folder_path, 'collision.obj')
            collision_shape_file = os.path.join(folder_path, 'collision.obj')
            texture_path = os.path.join(folder_path, 'texture_map.jpg')
            save_path = save_directory
            save_each_model = os.path.join(save_path, subdirectory)
            if not os.path.exists(save_each_model):
                os.makedirs(save_each_model)
            
            print(visual_shape_file)
            
            pb.setRealTimeSimulation(1)

            viewMatrix = pb.computeViewMatrix(
            cameraEyePosition=[1.0, 0 ,1.5],
                cameraTargetPosition=[0, 0, 0.39],
                cameraUpVector=[-1, 0, 0])

            projectionMatrix = pb.computeProjectionMatrixFOV(
                fov=65.0,
                aspect=1.0,
                nearVal=0.1,
                farVal=3.1)
            pb.setGravity(0, 0, -9.8)
            mesh_file = os.path.join(folder_path, 'visual.ply')
            
            for i in range(0, 360, 5):
                rand = round(random.uniform(i,i+5), 2)
                rotation = rand/360.  *np.pi * 2
                print(rotation)
                print("iter rot", rand)
            
                multiBodyId = object_setup(visual_shape_file, collision_shape_file, texture_path, rotation)
                pb.setGravity(0, 0, -9.8)
                mesh = o3d.io.read_triangle_mesh(mesh_file)
                pcd = mesh.sample_points_poisson_disk(2000, init_factor=5, pcl=None)
                # frame_init = o3d.geometry.TriangleMesh.create_coordinate_frame()
                # frame_init_r = copy.deepcopy(frame_init)
                
                R = pcd.get_rotation_matrix_from_xyz((0, 0, rotation))

                # print(R)
                pcd = pcd.rotate(R, center=True)
                # frame_init_r.rotate(R, center=True)
                # original_mat = np.loadtxt("/root/ocrtoc_ws/src/rendering_setup_iros22/saved_models/clear_box_rotation.txt").reshape(3, 3)
                # print(original_mat)
                # o3d.visualization.draw_geometries([pcd,frame_init,frame_init_r  ])
            
                time.sleep(0.5)
                width, height, rgbImg, depthImg, segImg = pb.getCameraImage(
                width=1024, 
                height=798,
                viewMatrix=viewMatrix,
                projectionMatrix=projectionMatrix)
            
                #Save stuff
            
                #rotation_matrix
                rotation_mat_file = os.path.join(save_each_model, "%02f.txt" % rand) 
                print(rotation_mat_file)
                rotation_mat = open(rotation_mat_file, "w+")
                for row in R:
                    np.savetxt(rotation_mat, row)
                rotation_mat.close()

                #save pcd
                pcd_file = os.path.join(save_each_model, "%02f.pcd" % rand) 
                o3d.io.write_point_cloud(pcd_file, pcd)
                
                #save image
                image_file = os.path.join(save_each_model, "%02f.png" % rand) 
                cv2.imwrite(image_file, cv2.cvtColor(rgbImg,  cv2.COLOR_RGB2BGR))
        
                pb.stepSimulation()
                delete_object(multiBodyId)
                pb.stepSimulation()
                time.sleep(1./240)
                

if __name__ == '__main__':
    main()
    print("Done!!!!!")
    while 1:
        pb.stepSimulation()
        time.sleep(1./240)