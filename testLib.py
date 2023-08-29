import laspy
import open3d as o3d
import numpy as np
from alphashape import alphashape
import matplotlib.pyplot as plt
import triangulate

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

minZ = np.PINF
maxZ = np.NINF

for point in point_data:
    if point[2] < minZ:
        minZ = point[2]
    if point[2] > maxZ:
        maxZ = point[2]

#print(1-1/len(point_data))
alpha = 0.5
print(point_data)
alphashapeTree = alphashape(point_data[:,:-1], alpha)
#alphashapeTree.remove_duplicate_faces()
#alphashapeTree.remove_degenerate_faces()

x,y = alphashapeTree.exterior.xy


# plt.plot(x,y)
# plt.show()

triangles = triangulate.triangulate(alphashapeTree.exterior.coords[:-1])

index = 1
vertices = {}
obj = ""

for tri in triangles:
    for vertex in tri:
        if vertex not in vertices:
            obj += "v " + "{:.8f}".format(vertex[0]) + " " + "{:.8f}".format(vertex[1]) + " " + "{:.8f}".format(maxZ) + "\n"
            vertices[vertex] = index
            index += 1


for coord in vertices.keys():
    obj += "v " + "{:.8f}".format(coord[0]) + " " + "{:.8f}".format(coord[1]) + " " + "{:.8f}".format(minZ) + "\n"

    
for tri in triangles:
    obj += "f"
    for vertex in tri:
        obj += " " + str(vertices[vertex])
    obj += "\n" 


length = len(vertices.keys())


for i in vertices.values():
    print(length)
    if i == length:
        obj += "f " + str(i) + " " + str(i + length) + " " + str(length+1) + "\n"
        obj += "f " + str(i) + " " + str(length+1) + " " + str(1) + "\n"
    else:
        obj += "f " + str(i) + " " + str(i + length) + " " + str((i + 1) + length) + "\n"
        obj += "f " + str(i) + " " + str((i + 1) + length) + " " + str((i + 1)) + "\n"

print(obj)

with open("./test.obj", 'w') as f:
        for line in obj:
            f.write(line)



# alphashapeTree.fill_holes()

"""
colors = []
intersect = set(point_data).intersection(alphashapeTree.vertices)
print(intersect)
for point in alphashapeTree.vertices:
    i = np.where(point_data == point)[0][0]
    colors.append(point_color[i])

alphashapeTree.visual.vertex_colors = colors
"""

#alphashapeTree.show()

#alphashapeTree.export("./test.obj")


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