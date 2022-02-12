import numpy as np
import open3d as o3d
import cv2
import copy, os
from mesh2pcd import pcd_sampler
import random

class DataGen():
    def __init__(self):
        #variables
        self.img_width = 640
        self.img_height = 360
        self.render = o3d.visualization.rendering.OffscreenRenderer(self.img_width, self.img_height)
        self.mtl = o3d.visualization.rendering.Material()
        self.mtl.shader = "defaultUnlit"
        
        self.done_list = []
        self.fail_list = [] 
        
        self.directory = '/root/ocrtoc_ws/src/rendering_setup_iros22/training_models'
        self.save_directory = '/root/ocrtoc_ws/src/rendering_setup_iros22/data_models'


    def unit_test(self, meshpath , texturepath):
        
        #https://github.com/Xelawk/mesh_utils
        
        mesh = o3d.io.read_triangle_mesh(meshpath)
        xyzs = np.asarray(mesh.vertices)
        nxyzs = np.asarray(mesh.vertex_normals)
        uvs = np.asarray(mesh.triangle_uvs)
        faces = np.asarray(mesh.triangles)

        # creating mesh_file and face_file self-defined
        mesh_file = "/root/ocrtoc_ws/src/rendering_setup_iros22/mesh_file"
        face_file = "/root/ocrtoc_ws/src/rendering_setup_iros22/face_file"

        with open(mesh_file, 'w') as f:
            for xyz, nxyz, uv in zip(xyzs, nxyzs, uvs):
                xyz_str = " ".join([str(x) for x in xyz.tolist()])
                nxyz_str = " ".join([str(x) for x in nxyz.tolist()])
                uv_str = " ".join([str(x) for x in uv.tolist()])
                data_str = [xyz_str, nxyz_str, uv_str]
                line = ','.join(data_str)
                f.write(line+'\n')

        with open(face_file, 'w') as f:
            for face in faces:
                data = face.tolist()
                data_str = [str(x) for x in data]
                line = ' '.join(data_str)
                f.write(line+'\n')

        pcd_obj = pcd_sampler.PcdSampler(
            mesh_file=mesh_file,
            face_file=face_file,
            img_file=texturepath
        )
        # try:
        pcd, colors = pcd_obj.sample_surface_even(50000)  # given sampling points you want
        # except:
        #     print("IndexError: index 417 is out of bounds for axis 0 with size 417")
        #     print("Sampling with 25000 points")
        #     pcd, colors = pcd_obj.sample_surface_even(416)
        pcd = pcd_obj.show_points_cloud(pcd, colors, show=False)  # show pointcloud wiht color sampling
        # pcd = o3d.geometry.trimesh.scale(0.5 , (0,0,0))
        
        return pcd

    def pcd_rotate_setup(self, rotation):
        
        pcd = self.unit_test(self.mesh_file, self.texture_path)
        roll = rotation[0] / 360. * (2*np.pi)
        pitch = rotation[1] / 360. * (2*np.pi)
        yaw = (rotation[2] / 360.) * (2*np.pi)
        # print(roll, pitch, yaw)
        R = pcd.get_rotation_matrix_from_xyz((roll, pitch, yaw))
        # print(R)
        pcd = pcd.rotate(R, center=(0, 0, 0))
        return pcd , R

    def camera_render_setup(self, pcd):
        self.render.scene.set_background([0, 0, 0, 0])
        self.render.scene.add_geometry("model", pcd , self.mtl)

        # intrinsics = np.array([[917.9434734500945, 0.0, 639.5], [0.0, 917.9434734500945, 359.5], [0, 0, 1]])
        # near_plane = 0.1
        # far_plane = 100.0

        # self.render.scene.camera.set_projection(intrinsics, near_plane, far_plane, self.img_width, self.img_height)
        
        #Sets the position and orientation of the camera: 
        #look_at(center, eye, up)
        self.camera_position = [0.2, 0.3, 0.15]
        self.render.scene.camera.look_at([0, 0, 0], self.camera_position, [0, 0, 1]) # what parameters should I give here ?
        img_o3d = self.render.render_to_image()

        img = np.array(img_o3d)

        self.render.scene.remove_geometry('model')
        return img

    def mask_pcd(self, pcd):
        
        diameter = np.linalg.norm(
            np.asarray(pcd.get_max_bound()) - np.asarray(pcd.get_min_bound()))
        # o3d.visualization.draw_geometries([pcd])
        radius = diameter * 100
        
        # print("Get all points that are visible from given view point")
        _, pt_map = pcd.hidden_point_removal(self.camera_position, radius)
        pcd_seen = pcd.select_by_index(pt_map)

        return pcd_seen






    def save_pcd(self, pcd):

        pcd_combined = self.mask_pcd(pcd)
        
        
        pcd = pcd
        points = np.asarray(pcd.points)
        colors = np.asarray(pcd.colors)
        arr_app = np.empty((0,7))
        for point,color in zip(points,colors):
            arr1 = point
            arr2 = color
            # print(arr1,arr2)
            
            if point in pcd_combined.points:
                               
                mask = [1]
                
            else:
                mask = [0]
                
            arr = np.concatenate([arr1, mask, arr2])
            arr_app = np.append(arr_app, np.array([arr]), axis=0)
                  
            
        return arr_app


    
    def save_data(self, img, pcd, R,  each_model_path, rotation ):
        
        rand = rotation[2]
        
        folder = os.path.join(each_model_path, self.subdirectory + '_' + str(int(str(rand).replace('.', ''))))
        if not os.path.exists(folder):
            os.makedirs(folder)

        #save_rotation
        rotation_mat_file = os.path.join(folder, "cls.txt") 
        rotation_mat = open(rotation_mat_file, "w+")
        for row in R:
            np.savetxt(rotation_mat, row)
        rotation_mat.close()

        #save_pcd
        pcd_file = os.path.join(folder, "pointcloud.npz") 
        pcd_array = self.save_pcd(pcd)
        np.savez_compressed(pcd_file, data = pcd_array)

        #save img
        image_file = os.path.join(folder, "projection.png") 
        cv2.imwrite(image_file, cv2.cvtColor(img, cv2.COLOR_BGR2RGB))

        #img mask
        mask_img_file = os.path.join(folder, "mask.npy") 
        mask_gray =  cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        (thresh, mask) = cv2.threshold(mask_gray, 128, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        # cv2.imwrite(mask_img_file, mask)
        np.save( mask_img_file, mask)
        print("saved data", each_model_path, rand)


    def data_gen(self, each_model_path):
        
        for roll in range(0, 360, 90):
            for pitch in range(0, 360, 45):
                for yaw in range(0, 360, 20):
                    
                    if roll + 90 < 360:
                        roll_rand = round(random.uniform(roll,roll + 90), 3)
                    else:
                        roll_rand = round(random.uniform(roll,360), 3)
                    
                    if pitch + 45 < 360:
                        pitch_rand = round(random.uniform(pitch, pitch + 45), 3)
                    else:
                        pitch_rand = round(random.uniform(pitch, 360), 3)
                    
                    if yaw + 20 < 360:
                        yaw_rand = round(random.uniform(yaw,yaw+20), 3)
                    else:
                        yaw_rand = round(random.uniform(yaw,360), 3)
                    

                    rotation = [roll_rand,pitch_rand,yaw_rand]
                    pcd_r, R = self.pcd_rotate_setup(rotation)
                    img = self.camera_render_setup(pcd_r)

                    #save stuff
                    self.save_data(img, pcd_r, R, each_model_path, rotation)

    def main(self):
        # self.done_list = ['stapler_1', 'conditioner', 'spoon', 'yellow_bowl', 

        # ]

        issue_list = []
        count = 0
        
        for root, subdirectories, files in os.walk(self.directory):
            for self.subdirectory in subdirectories:
                
                if self.subdirectory in self.done_list or self.subdirectory in self.fail_list:
                    continue
                print("Objects done: ", self.done_list)
                print("Objects Failed: ", self.fail_list)
                print(os.path.join(root, self.subdirectory))
                # print(subdirectories)

                folder_path = os.path.join(root, self.subdirectory)
                self.mesh_file = os.path.join(folder_path, 'textured.obj')

                if os.path.exists(os.path.join(folder_path, 'texture_map.jpg')):
                    self.texture_path = os.path.join(folder_path, 'texture_map.jpg')
                elif os.path.exists(os.path.join(folder_path, 'texture_map.png')):
                    self.texture_path = os.path.join(folder_path, 'texture_map.png')
                    count = count + 1
                    issue_list.append(self.subdirectory )

                else:
                    self.texture_path = os.path.join(folder_path, 'textured_map.jpg')
                    print(folder_path)
                    print(1)

                each_model_path = self.save_directory
                
                try:
                    # self.data_gen(each_model_path)
                    self.done_list.append(self.subdirectory)
                    print("Done: ", self.subdirectory )
                except:
                    self.fail_list.append(self.subdirectory)
                    print("Failed: ", self.subdirectory)

            print(count)
        print(issue_list)
                
                

if __name__ == '__main__':
    gen = DataGen()
    gen.main()
