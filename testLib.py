import laspy
import open3d as o3d
import numpy as np
from alphashape import alphashape

las = laspy.read('ExampleDataCutCut.las')

new_file = laspy.create(point_format=las.header.point_format, file_version=las.header.version)
new_file.points = las.points[las.classification == 5]

point_data = np.stack([new_file.x, new_file.y, new_file.z], axis=0).transpose((1, 0))


las.red = las.red*(255.0/las.red.max())
las.green = las.green*(255.0/las.green.max())
las.blue = las.blue*(255.0/las.blue.max())


point_color = np.stack([las.red, las.green, las.blue, np.full(las.blue.shape, 255)], axis=0).transpose((1, 0))

geom = o3d.geometry.PointCloud()
geom.points = o3d.utility.Vector3dVector(point_data)


#print(1-1/len(point_data))
alpha = 0.001
alphashapeTree = alphashape(point_data, alpha)
#alphashapeTree.remove_duplicate_faces()
#alphashapeTree.remove_degenerate_faces()
alphashapeTree.fill_holes()
alphashapeTree.fix_normals()
colors = []
for point in alphashapeTree.vertices:
    i = np.where(point_data == point)[0][0]
    colors.append(point_color[i])

alphashapeTree.visual.vertex_colors = colors

alphashapeTree.show()

alphashapeTree.export("./test.obj")


"""
hull, _ = geom.compute_convex_hull()
#o3d.visualization.draw_geometries([hull])
o3d.io.write_triangle_mesh("hullTest.ply", hull)
"""

"""
mesh = o3d.geometry.TriangleMesh.create_sphere()
pcd = mesh.sample_points_poisson_disk(750)
#o3d.visualization.draw_geometries([geom])
alpha = 9000000000000
print(f"alpha={alpha:.3f}")
#tetra_mesh, pt_map = o3d.geometry.TetraMesh.create_from_point_cloud(pcd)
#o3d.visualization.draw_geometries([geom], mesh_show_back_face=True)
mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(geom, alpha)
mesh.compute_vertex_normals()
#mesh, _ = geom.compute_convex_hull()
o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)
"""

# o3d.visualization.draw_geometries([geom])

# geom = geom.voxel_down_sample(voxel_size=80)
# o3d.visualization.draw_geometries([geom])
# geom.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=40, max_nn=30))
# geom.estimate_normals()

# o3d.visualization.draw_geometries([geom], point_show_normal=True)

"""
Ne fonctionne pas, peut être qu'un mesh avec des "trous" ne fonctionne pas avec cet algo

Testé avec un abre isolé, même erreur

alpha = 0.03
tetra_mesh, pt_map = o3d.geometry.TetraMesh.create_from_point_cloud(geom)
mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(geom, alpha, tetra_mesh, pt_map)
mesh.compute_triangle_normals(normalized=True)
o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)
"""


"""
with o3d.utility.VerbosityContextManager(
        o3d.utility.VerbosityLevel.Debug) as cm:
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(geom, depth=10)

densities = np.asarray(densities)

vertices_to_remove = densities < np.quantile(densities, 0.2)
mesh.remove_vertices_by_mask(vertices_to_remove)
"""

"""
mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(geom, depth=8, width=0, scale=1.1, linear_fit=False)[0]
bbox = geom.get_axis_aligned_bounding_box()
p_mesh_crop = mesh.crop(bbox)
o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)
"""


"""
# new_file.write("filtered.las")


o3d.io.write_triangle_mesh(filename="mesh.obj", mesh=mesh, write_vertex_normals=False, write_vertex_colors=False, write_triangle_uvs=False, print_progress=True, write_ascii=True)
"""