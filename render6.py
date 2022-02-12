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
    pcd, colors = pcd_obj.sample_surface_even(25000)  # given sampling points you want
    pcd = pcd_obj.show_points_cloud(pcd, colors, show=False)  # show pointcloud wiht color sampling
    # pcd = o3d.geometry.trimesh.scale(0.5 , (0,0,0))
    
    return pcd


img_width = 640
img_height = 480
render = o3d.visualization.rendering.OffscreenRenderer(img_width, img_height)
folder_path = "/root/ocrtoc_ws/src/rendering_setup_iros22/training_models/orion_pie"

texture_path = os.path.join(folder_path, 'texture_map.jpg')
mesh_file = os.path.join(folder_path, 'textured.obj')

#get mesh with texture, 
#else just read the mesh file into pcd. 
pcd = unit_test(mesh_file, texture_path) 
# o3d.visualization.draw_geometries([pcd])
mesh = pcd
# mesh = o3d.io.read_triangle_mesh(mesh_file)
R = mesh.get_rotation_matrix_from_xyz((0 , 0, 0 ))
mesh_r = copy.deepcopy(mesh)
mesh_r.rotate(R, center=(0, 0, 0))

mesh_r = mesh_r.translate((0.0, 0.0, -0.2))

render.scene.set_background([0, 0, 0, 0])


mtl = o3d.visualization.rendering.Material()
# mtl.base_color = [1.0, 1.0, 1.0, 1.0]  # RGBA
mtl.shader = "defaultUnlit"
# render.scene.add_geometry("rotated_model", mesh_r, mtl)
render.scene.add_geometry("model", mesh_r , mtl)
# 
render.scene.camera.look_at([0, 0, 0], [0.2, 0.3, 0.15], [0, 0, 1])
     # what parameters should I give here ?

#projection matrix
print(render.scene.camera.get_projection_matrix())

camera = [0.1, 0.15, 0.1]

diameter = np.linalg.norm(
    np.asarray(pcd.get_max_bound()) - np.asarray(pcd.get_min_bound()))
# o3d.visualization.draw_geometries([pcd])
radius = diameter * 100

print("Get all points that are visible from given view point")
points, pt_map = pcd.hidden_point_removal(camera, radius)
# print(points)

print("Visualize result")
# print(pt_map)
print(pcd)
pcd1 = pcd.select_by_index(pt_map, invert=True)


# o3d.visualization.draw_geometries([pcd1])
# print(pcd1)
# np_colors = np.array(pcd1.colors)
# np_colors[3000:45000,:] = 0.5
# pcd.colors = o3d.utility.Vector3dVector(np_colors)
# o3d.visualization.draw_geometries([pcd])



np_colors = np.array(pcd.select_by_index(pt_map, invert=True).colors)
# print(np_colors)
print(len(np_colors))
np_colors[:,:] = 0.0
pcd.select_by_index(pt_map, invert=True).colors = o3d.utility.Vector3dVector(np_colors)

pcd1 = pcd.select_by_index(pt_map, invert=False)
pcd1.paint_uniform_color([0.7, 0.7, 0.7])

pcd2 = pcd.select_by_index(pt_map, invert=True)
pcd2.paint_uniform_color([0.0, 0.0, 0.0])

# o3d.visualization.draw_geometries([pcd1])

pcd3 = pcd2
pcd3 += pcd1



points = np.asarray(pcd.points)
colors = np.asarray(pcd.colors)
print(pcd.select_by_index(pt_map))
print(points[0])
print(colors[0])
count  =0 
arr = None
arr_app = np.empty((0,7))

filename = '/root/ocrtoc_ws/src/rendering_setup_iros22/test.npz'


# for point,color in zip(points,colors):
#     arr1 = point
#     arr2 = color
#     # print(arr1,arr2)
    
#     if point in pcd1.points:
#         # print(True)
        
#         mask = [1]
#         # arr1.append(mask)
#         # arr1.append(arr2)
        
#         arr = np.concatenate([arr1, mask, arr2])
#         arr1 = arr
#         arr_app = np.append(arr_app, np.array([arr]), axis=0)
#         # np.vstack((arr_app, arr))
#         # np.savez(filename, points = arr1, mask = 0, colors = arr2 )
#         # np.concatenate((arr_app, arr), axis= 1)
#         # arr_app = np.append(arr_app[count], arr, axis = 1)
#         # arr = np.append(arr1, mask, arr2)   
#     count = count+1

# print(count)
# print(pcd1.points)
# print(arr)
# print(arr_app[0])
# print(arr_app[:, 3])

# np.savez(filename, arr_app)
# with np.load(filename) as data:
#     print(data[0])
# print(arr[1])


print('xyz_load')
# print(colors)


img_o3d = render.render_to_image()
print("img_o3d", img_o3d)
img = np.array(img_o3d)
# np.save( filename + '.npy', data) for mask


img = cv2.cvtColor(np.array(img_o3d), cv2.COLOR_BGR2RGB )
cv2.imshow("foot_model", img)
cv2.waitKey()
