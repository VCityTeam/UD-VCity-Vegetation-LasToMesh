from os import listdir, remove, mkdir
from os.path import isfile, join
from alphashape import alphashape

import numpy as np
import open3d as o3d
import time
import math


def pointToColor(point):
    return str(point[0])+str(point[1])+str(point[2])

def clearFolders(path):

    try:
        onlyfiles = [f for f in listdir(path) if isfile(join(path, f))]
        for file in onlyfiles:
            remove(path+str(file))
    except FileNotFoundError:
        mkdir(path)
    

    
def exportHullInOBJ(vertices, colors, triangles, outputpath):
    
    obj = ""
    
    # Vertex coordinates + color
    for i in range(len(vertices)):
        # Formating to always have 8 digits in the decimals
        obj += "v " + "{:.8f}".format(vertices[i][0]) + " " + "{:.8f}".format(vertices[i][1]) + " " + "{:.8f}".format(vertices[i][2]) + " "
        obj += "{:.8f}".format(colors[i][0]) + " " + "{:.8f}".format(colors[i][0]) + " " + "{:.8f}".format(colors[i][0]) + "\n"

    # Triangles
    for face in triangles:
        # Careful : the index start at 0 in the input but must start at 1 in the output
        obj += "f " + str(face[0]+1) + " " + str(face[1]+1) + " " + str(face[2]+1) + "\n"

    # Output
    with open(outputpath, 'w') as f:
        for line in obj:
            f.write(line)

def createConvexHull(pointCloud, colors_normalized, path):
    # Creating the hull
    geom = o3d.geometry.PointCloud()

    geom.points = o3d.utility.Vector3dVector(pointCloud)
    hull, _ = geom.compute_convex_hull()
    
    # Reconnecting the color to the vertices
    pointColors = []
    for vertice in np.asarray(hull.vertices):
        pointColors.append(colors_normalized[pointToColor(vertice)])
    hull.vertex_colors = o3d.utility.Vector3dVector(pointColors)

    #o3d.visualization.draw_geometries([hull])
    
    exportHullInOBJ(hull.vertices, hull.vertex_colors, hull.triangles, path)


def computeAlpha(pointCloud):
    # Parameter between 1 and 0 dictating the level of detail of the alpha shape.
    # The closer you are to 1 the more details you'll have, risking having holes in the mesh.
    # The closer you are to 0, the closer you'll be from the convex hull.
    # The formula bellow reduce the alpha the more points you have in the island.
    alpha = 1-(math.log(len(pointCloud), 10)*1.8)/10
    if(alpha < 0.01):
        alpha = 0.01
    print("len : "+str(len(pointCloud)))
    print("alpha : " + str(alpha))

    return alpha

def createAlphashape(pointCloud, alpha, colors, path):
    
    start = time.time()
    print("Starting creation of alpha shape")
    alphashapeTree = alphashape(pointCloud, alpha)
    end = time.time()
    print("Finished creation of alpha shape in " + str(end - start) + " seconds") # time in seconds
    
    # Doesn't seem to affect to mesh much but isn't costly
    alphashapeTree.fill_holes()
    # Seems to only flip the normals, doesn't detect fine normals
    #alphashapeTree.fix_normals()
    end = time.time()
    
    start = time.time()
    print("Starting coloring alpha shape")
    # Reconnecting the color to the vertices
    pointColors = []
    for vertice in alphashapeTree.vertices:
        pointColors.append(colors[pointToColor(vertice)])
    alphashapeTree.visual.vertex_colors = pointColors
    end = time.time()
    print("Finished coloring alpha shape in " + str(end - start) + " seconds") # time in seconds

    alphashapeTree.export(path)
    return
