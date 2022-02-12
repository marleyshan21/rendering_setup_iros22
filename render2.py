import numpy as np
import open3d as o3d
import cv2
import copy, os
from mesh2pcd import pcd_sampler



def unit_test(meshpath , texturepath):
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
    pcd, colors = pcd_obj.sample_surface_even(50000)  # given sampling points you want
    pcd = pcd_obj.show_points_cloud(pcd, colors, show=True)  # show pointcloud wiht color sampling
    # pcd = o3d.geometry.trimesh.scale(0.5 , (0,0,0))
    
    return pcd


img_width = 640
img_height = 480
render = o3d.visualization.rendering.OffscreenRenderer(img_width, img_height)
folder_path = "/root/ocrtoc_ws/src/rendering_setup_iros22/training_models/cleanser"
# /home/vishal_mandadi/iros_rendering/rendering_setup_iros22/training_models/plastic_banana

texture_path = os.path.join(folder_path, 'textured_map.jpg')
mesh_file = os.path.join(folder_path, 'textured.obj')

#get mesh with texture, 
#else just read the mesh file into pcd. 
pcd = unit_test(mesh_file, texture_path) 
o3d.visualization.draw_geometries([pcd])
mesh = pcd
# mesh = o3d.io.read_triangle_mesh(mesh_file)
R = mesh.get_rotation_matrix_from_xyz((np.pi / 2, 0, np.pi / 4))
mesh_r = copy.deepcopy(mesh)
mesh_r.rotate(R, center=(0, 0, 0))
render.scene.set_background([0, 0, 0, 0])


mtl = o3d.visualization.rendering.Material()
# mtl.base_color = [1.0, 1.0, 1.0, 1.0]  # RGBA
mtl.shader = "defaultUnlit"
# render.scene.add_geometry("rotated_model", mesh_r, mtl)
# render.scene.add_geometry("model", mesh , mtl)

# render.scene.camera.look_at([0, 0, 0], [0.1, 0.15, 0], [0, 0, 1]) # what parameters should I give here ?

render.scene.set_background([0, 0, 0, 0])
render.scene.add_geometry("model", pcd , mtl)

# intrinsics = np.array([[917.9434734500945, 0.0, 639.5], [0.0, 917.9434734500945, 359.5], [0, 0, 1]])
# near_plane = 0.1
# far_plane = 100.0

# render.scene.camera.set_projection(intrinsics, near_plane, far_plane, img_width, img_height)

#Sets the position and orientation of the camera: 
#look_at(center, eye, up)
camera_position = [0.2, 0.3, 0.15]
render.scene.camera.look_at([0, 0, 0], camera_position, [0, 0, 1]) # what parameters should I give here ?
img_o3d = render.render_to_image()


#projection matrix
print("projection matrix::::::::::")
print(render.scene.camera.get_projection_matrix())

img_o3d = render.render_to_image()
print("img_o3d", img_o3d)
img = np.array(img_o3d)
img = cv2.cvtColor(np.array(img_o3d), cv2.COLOR_BGR2RGB )
cv2.imshow("foot_model", img)
cv2.waitKey()
