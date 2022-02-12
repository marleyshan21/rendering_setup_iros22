import time
import pybullet as pb
physicsClient = pb.connect(pb.DIRECT)
import os
import copy
import cv2
import pybullet_data
import open3d as o3d
import numpy as np
import random

directory = '/root/ocrtoc_ws/src/rendering_setup_iros22/training_models'
save_directory = '/root/ocrtoc_ws/src/rendering_setup_iros22/saved_models'

# def save_data():
def object_setup(visual_shape_file, collision_shape_file, texture_path, rotation, z_pos):
    
    # print(yaw)
    
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
        basePosition=[0.0, 0.0, z_pos],
        baseOrientation=pb.getQuaternionFromEuler([0, rotation[0], rotation[1]]))
    
    
    textureId = pb.loadTexture(texture_path)
    pb.changeVisualShape(multiBodyId, -1, textureUniqueId=textureId)
    return multiBodyId

def delete_object(multiBodyId):
    
    pb.removeBody(multiBodyId)

def main():

    root = '/root/ocrtoc_ws/src/rendering_setup_iros22/training_models'
    subdirectories = ['stapler_1', 'conditioner', 'spoon', 'pen_container_1', 'yellow_bowl', 
    'bleach_cleanser', 'plate_holder', 'yellow_cup', 'blue_plate', 'power_drill', 'blue_tea_box', 
    'potato_chip_1', 'extra_large_clamp', 'round_plate_1', 'shampoo', 'phillips_screwdriver', 'book_3',
     'round_plate_3', 'fork', 'plastic_apple', 'soap', 'doraemon_plate', 'repellent', 'suger_1', 
     'book_holder_1', 'blue_marker', 'mug', 'orion_pie', 'lipton_tea', 'bowl', 'square_plate_1', 
     'potato_chip_2', 'cracker_box', 'clear_box_2', 'black_marker', 'book_2', 'plastic_orange', 
     'small_clamp', 'glue_1', 'poker_1', 'plastic_peach', 'two_color_hammer', 'plastic_pear', 
     'doraemon_bowl', 'cleanser', 'suger_2', 'soap_dish', 'correction_fuid', 'flat_screwdriver',
      'book_holder_2', 'square_plate_2', 'plastic_banana', 'book_4', 'plastic_lemon', 'blue_moon',
       'book_1', 'remote_controller_1', 'orange_cup', 'round_plate_2', 'square_plate_3', 'scissors', 
       'pudding_box', 'large_marker', 'toothpaste_1', 'grey_plate', 'large_clamp', 'knife', 
       'clear_box', 'pitcher', 'pink_tea_box']
    
    
    #plate_holder, power_drill, shampoo, book_3, orion_pie, book_2, book_4 - issue - needs roll
    # round_plate_1, round_plate_3 - symmetry 
       
    subdirectory_dict = { 'stapler_1': { 'pitch_req': [0, np.pi/2], "z_pos_yaw": 0.82 , "z_pos_pitch": 0.82},
    
    'conditioner': { 'pitch_req': [0, np.pi/2], "z_pos_yaw": 0.85 , "z_pos_pitch": 0.82},
    'spoon': { 'pitch_req': [0], "z_pos_yaw": 0.82 , "z_pos_pitch": 0.82},
    'pen_container_1':{ 'pitch_req': [0, np.pi/2], "z_pos_yaw": 0.82 , "z_pos_pitch": 0.9},
    'yellow_bowl':{ 'pitch_req': [0, np.pi], "z_pos_yaw": 0.82 , "z_pos_pitch": 0.85},
    'bleach_cleanser':{ 'pitch_req': [0, np.pi/2], "z_pos_yaw": 0.9 , "z_pos_pitch": 0.82},
    'yellow_cup':{ 'pitch_req': [0, np.pi], "z_pos_yaw": 0.82 , "z_pos_pitch": 0.82},
    'blue_plate':{ 'pitch_req': [0, np.pi], "z_pos_yaw": 0.82 , "z_pos_pitch": 0.82},
    'power_drill':{ 'pitch_req': [0, np.pi/2], "z_pos_yaw": 0.88 , "z_pos_pitch": 0.82},
    'blue_tea_box':{ 'pitch_req': [0, np.pi/2], "z_pos_yaw": 0.82 , "z_pos_pitch": 0.82},
    'potato_chip_1':{ 'pitch_req': [0, np.pi/2], "z_pos_yaw": 0.85 , "z_pos_pitch": 0.82},
    'extra_large_clamp':{ 'pitch_req': [0], "z_pos_yaw": 0.85 , "z_pos_pitch": 0.82},
    'round_plate_1':{ 'pitch_req': [0], "z_pos_yaw": 0.82 , "z_pos_pitch": 0.82},
    'shampoo':{ 'pitch_req': [np.pi/2], "z_pos_yaw": 0.82 , "z_pos_pitch": 0.82},
    'phillips_screwdriver':{ 'pitch_req': [0], "z_pos_yaw": 0.78 , "z_pos_pitch": 0.82},
    'round_plate_3':{ 'pitch_req': [0], "z_pos_yaw": 0.78 , "z_pos_pitch": 0.82},
    'fork':{ 'pitch_req': [0], "z_pos_yaw": 0.82 , "z_pos_pitch": 0.82},
    'plastic_apple':{ 'pitch_req': [0], "z_pos_yaw": 0.80 , "z_pos_pitch": 0.82},
    'soap':{ 'pitch_req': [0], "z_pos_yaw": 0.80 , "z_pos_pitch": 0.82},
    'doraemon_plate':{ 'pitch_req': [0], "z_pos_yaw": 0.80 , "z_pos_pitch": 0.82},
    'repellent':{ 'pitch_req': [0, np.pi/2], "z_pos_yaw": 0.88 , "z_pos_pitch": 0.82},
    'suger_1':{ 'pitch_req': [0, np.pi/2], "z_pos_yaw": 0.82 , "z_pos_pitch": 0.82},
    'book_holder_1':{ 'pitch_req': [0], "z_pos_yaw": 0.88 , "z_pos_pitch": 0.88},
    'blue_marker':{ 'pitch_req': [0], "z_pos_yaw": 0.78 , "z_pos_pitch": 0.88},
    'mug':{ 'pitch_req': [0], "z_pos_yaw": 0.84 , "z_pos_pitch": 0.88},
    'orion_pie':{ 'pitch_req': [0, np.pi/2], "z_pos_yaw": 0.84 , "z_pos_pitch": 0.88},
    'lipton_tea':{ 'pitch_req': [0, np.pi/2], "z_pos_yaw": 0.84 , "z_pos_pitch": 0.88},
    'bowl':{ 'pitch_req': [0, np.pi], "z_pos_yaw": 0.84 , "z_pos_pitch": 0.88},
    'square_plate_1':{ 'pitch_req': [0], "z_pos_yaw": 0.78 , "z_pos_pitch": 0.82},
    'potato_chip_2':{ 'pitch_req': [0, np.pi/2], "z_pos_yaw": 0.85 , "z_pos_pitch": 0.82},
    'cracker_box':{ 'pitch_req': [0, np.pi/2], "z_pos_yaw": 0.85 , "z_pos_pitch": 0.88},
    'clear_box_2':{ 'pitch_req': [0,  np.pi/2], "z_pos_yaw": 0.85 , "z_pos_pitch": 0.88},
    'black_marker':{ 'pitch_req': [0], "z_pos_yaw": 0.78 , "z_pos_pitch": 0.88},
    'plastic_orange':{ 'pitch_req': [0], "z_pos_yaw": 0.80 , "z_pos_pitch": 0.82},
    'small_clamp':{ 'pitch_req': [0], "z_pos_yaw": 0.85 , "z_pos_pitch": 0.82},
    'glue_1':{ 'pitch_req': [0, np.pi/2], "z_pos_yaw": 0.85 , "z_pos_pitch": 0.82},
    'poker_1':{ 'pitch_req': [0, np.pi/2], "z_pos_yaw": 0.85 , "z_pos_pitch": 0.82},
    'plastic_peach':{ 'pitch_req': [0], "z_pos_yaw": 0.80 , "z_pos_pitch": 0.82},
    'two_color_hammer':{ 'pitch_req': [0], "z_pos_yaw": 0.85 , "z_pos_pitch": 0.82},
    'plastic_pear':{ 'pitch_req': [0], "z_pos_yaw": 0.80 , "z_pos_pitch": 0.82},
    'doraemon_bowl':{ 'pitch_req': [0, np.pi], "z_pos_yaw": 0.82 , "z_pos_pitch": 0.85},
    'cleanser':{ 'pitch_req': [0, np.pi/2], "z_pos_yaw": 0.88 , "z_pos_pitch": 0.82},
    'suger_2':{ 'pitch_req': [0, np.pi/2], "z_pos_yaw": 0.82 , "z_pos_pitch": 0.82},
    'soap_dish':{ 'pitch_req': [0], "z_pos_yaw": 0.80 , "z_pos_pitch": 0.82},
    'correction_fuid':{ 'pitch_req': [0], "z_pos_yaw": 0.78 , "z_pos_pitch": 0.88},
    'flat_screwdriver':{ 'pitch_req': [0], "z_pos_yaw": 0.78 , "z_pos_pitch": 0.82},
    'book_holder_2':{ 'pitch_req': [0], "z_pos_yaw": 0.88 , "z_pos_pitch": 0.88},
    'square_plate_2':{ 'pitch_req': [0], "z_pos_yaw": 0.78 , "z_pos_pitch": 0.82},
    'plastic_banana':{ 'pitch_req': [0], "z_pos_yaw": 0.80 , "z_pos_pitch": 0.82},
    'plastic_lemon':{ 'pitch_req': [0], "z_pos_yaw": 0.80 , "z_pos_pitch": 0.82},
    'blue_moon':{ 'pitch_req': [0, np.pi/2], "z_pos_yaw": 0.90 , "z_pos_pitch": 0.82},
    'remote_controller_1':{ 'pitch_req': [0], "z_pos_yaw": 0.78 , "z_pos_pitch": 0.88},
    'orange_cup':{ 'pitch_req': [0, np.pi], "z_pos_yaw": 0.82 , "z_pos_pitch": 0.82},
    'round_plate_2':{ 'pitch_req': [0], "z_pos_yaw": 0.78 , "z_pos_pitch": 0.82},
    'square_plate_3':{ 'pitch_req': [0], "z_pos_yaw": 0.78 , "z_pos_pitch": 0.82},
    'scissors':{ 'pitch_req': [0], "z_pos_yaw": 0.78 , "z_pos_pitch": 0.82},
    'pudding_box':{ 'pitch_req': [0, np.pi/2], "z_pos_yaw": 0.78 , "z_pos_pitch": 0.82},
    'large_marker':{ 'pitch_req': [0], "z_pos_yaw": 0.78 , "z_pos_pitch": 0.88},
    'toothpaste_1':{ 'pitch_req': [0], "z_pos_yaw": 0.78 , "z_pos_pitch": 0.88},
    'grey_plate':{ 'pitch_req': [0, np.pi], "z_pos_yaw": 0.82 , "z_pos_pitch": 0.82},
    'large_clamp':{ 'pitch_req': [0], "z_pos_yaw": 0.85 , "z_pos_pitch": 0.82},
    'knife':{ 'pitch_req': [0], "z_pos_yaw": 0.82 , "z_pos_pitch": 0.82},
    'clear_box':{ 'pitch_req': [0], "z_pos_yaw": 0.85 , "z_pos_pitch": 0.88},
    'pitcher':{ 'pitch_req': [0], "z_pos_yaw": 0.90 , "z_pos_pitch": 0.88},
    'pink_tea_box':{ 'pitch_req': [0, np.pi/2], "z_pos_yaw": 0.82 , "z_pos_pitch": 0.82}


    } 
    
    for obj in subdirectory_dict:
        subdirectory = obj
    # subdirectory = 'pink_tea_box' #subdirectory_dict[0]
        print(subdirectory)
        folder_path = os.path.join(root, subdirectory)
        visual_shape_file = os.path.join(folder_path, 'textured.obj')
        collision_shape_file = os.path.join(folder_path, 'collision.obj')

        if os.path.exists(os.path.join(folder_path, 'texture_map.jpg')):
            texture_path = os.path.join(folder_path, 'texture_map.jpg')
        elif os.path.exists(os.path.join(folder_path, 'texture_map.png')):
            texture_path = os.path.join(folder_path, 'texture_map.png')

        else:
            texture_path = os.path.join(folder_path, 'textured_map.jpg')
        save_path = save_directory
        save_each_model = os.path.join(save_path, subdirectory)
        if not os.path.exists(save_each_model):
            os.makedirs(save_each_model)
        
        # print(visual_shape_file)
        
        pb.setRealTimeSimulation(1)

        viewMatrix = pb.computeViewMatrix(
        cameraEyePosition=[0.30, 0 ,1.3],
            cameraTargetPosition=[-0.2, 0, 0.5],
            cameraUpVector=[-1, 0, 0])

        projectionMatrix = pb.computeProjectionMatrixFOV(
            fov=65.0,
            aspect=1.0,
            nearVal=0.1,
            farVal=3.1)
        pb.setGravity(0, 0, -9.8)
        mesh_file = os.path.join(folder_path, 'visual.ply')
        
        #choose the rotations based on symmetry and drop from a particular height
        

    
        print(subdirectory_dict[subdirectory]['pitch_req'])

        for pitch in subdirectory_dict[subdirectory]['pitch_req']: # , 0]:
            
            if pitch == 0:
                z_pos = subdirectory_dict[subdirectory]['z_pos_yaw']
            if pitch == np.pi/2 or pitch == -np.pi/2 :
                z_pos = subdirectory_dict[subdirectory]['z_pos_pitch']

            for i in range(0, 360, 5):

                pb.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"))
                pb.loadURDF("/root/ocrtoc_ws/src/rendering_setup_iros22/table/table.urdf")

                rand = round(random.uniform(i,i+5), 2)
                yaw = rand/360.  *np.pi * 2
                print(yaw)
                rotation = [pitch, yaw]
                print("iter rot", rand)
            
                multiBodyId = object_setup(visual_shape_file, collision_shape_file, texture_path, rotation, z_pos)
                pb.setGravity(0, 0, -9.8)
                mesh = o3d.io.read_triangle_mesh(mesh_file)
                pcd = mesh.sample_points_poisson_disk(2000, init_factor=5, pcl=None)
                frame_init = o3d.geometry.TriangleMesh.create_coordinate_frame()
                frame_init_r = copy.deepcopy(frame_init)
                
                R = pcd.get_rotation_matrix_from_xyz((0, pitch, yaw))

                # print(R)
                pcd = pcd.rotate(R, center=(0, 0, 0))
                frame_init_r.rotate(R, center=(0, 0, 0))
                # original_mat = np.loadtxt("/root/ocrtoc_ws/src/rendering_setup_iros22/saved_models/clear_box_rotation.txt").reshape(3, 3)
                # print(original_mat)
                # o3d.visualization.draw_geometries([pcd,frame_init,frame_init_r  ])
            
                
            
                #Save stuff
            
                #rotation_matrix
                rotation_mat_file = os.path.join(save_each_model, "%02f.txt" % rand) 
                print(rotation_mat_file)
                rotation_mat = open(rotation_mat_file, "w+")
                for row in R:
                    np.savetxt(rotation_mat, row)
                rotation_mat.close()

                #save pcd
                # pcd_file = os.path.join(save_each_model, "%02f.pcd" % rand) 
                # o3d.io.write_point_cloud(pcd_file, pcd)
                
                #save image
                time.sleep(0.5)
                width, height, rgbImg, depthImg, segImg = pb.getCameraImage(
                width=1024, 
                height=798,
                viewMatrix=viewMatrix,
                projectionMatrix=projectionMatrix)


                image_file = os.path.join(save_each_model, "%02f.png" % rand) 
                cv2.imwrite(image_file, cv2.cvtColor(rgbImg,  cv2.COLOR_RGB2BGR))

                pb.stepSimulation()
                delete_object(multiBodyId)
                pb.stepSimulation()
                time.sleep(1./240)

                # reset pybullet
                pb.resetSimulation()
                        

if __name__ == '__main__':
    main()
    print("Done!!!!!")
    