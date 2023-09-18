from os import listdir, remove, mkdir
from os.path import isfile, join
from alphashape import alphashape

import numpy as np
import open3d as o3d
import time
import math
import scipy
import trimesh
import triangulate
import meshCreation


def pointToColor(point):
    """ Returns a string that can be used as a key based on a point's coordinates (for example the color dict)
    
    Parameters
    ------
    point : array
      Array that contains the x, y and z coordinates of the point

    Returns
    -------
    index : str
        String that can be used as a key
    """
    return str(point[0])+str(point[1])+str(point[2])

def clearFolders(path):
    """ Clear all of the existing files inside a folder
    
    Parameters
    ------
    path : str
      Path to the folder
    """
    try:
        onlyfiles = [f for f in listdir(path) if isfile(join(path, f))]
        for file in onlyfiles:
            remove(path+str(file))
    except FileNotFoundError:
        mkdir(path)
    

    
def exportHullInOBJ(vertices, colors, triangles, outputpath):
    """ Create an obj file using vertices, colors and triangles provided
    
    Parameters
    ------
    vertices : 2d-array
      In column, index of the point.  
      In line, array that contains the x, y and z coordinates of the point.
    colors : 2d-array
      In column, index of the point.  
      In line, array that contains the r, g and b values of the point.
    triangles : 2d-array
      In column, index of the triangle.  
      In line, array that contains the indexes of points making the triangle.
    path : str
        Path to save the obj file

    """

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
    """ Create a convex hull mesh based on a provided point cloud and save it as an obj file
    
    Parameters
    ------
    pointCloud : 2d-array
        In column, index of the point.  
        In line, array that contains the x, y and z coordinates of the point.
    colors_normalized : 2d-array
        In column, index of the point.  
        In line, array that contains the r, g and b values of the point (between 0 and 1).
    path : str
        Path to save the obj file

    """
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
    """ Compute an alpha parameter for the given point cloud using its size
    
    Parameters
    ------
    pointCloud : 2d-array
        In column, index of the point.  
        In line, array that contains the x, y and z coordinates of the point.
        
    Returns
    -------
    alpha : float
        Value to provide to the alpha shape algorithm
    """
    # Parameter dictating the level of detail of the alpha shape.
    # The closer you are to 0, the closer you'll be from the convex hull.
    # The closer you are to Inf the more details you'll have, risking having holes in the mesh.
    # The formula bellow reduce the alpha the more points you have in the island.
    alpha = 1-(math.log(len(pointCloud), 10)*1.8)/10
    if(alpha < 0.01):
        alpha = 0.01
    meshCreation.log.info("len : "+str(len(pointCloud)))
    meshCreation.log.info("alpha : " + str(alpha))

    return alpha

def createExtruded2DAlphaShape(pointCloud, colors, alpha, path):
    """Create an extruded 2D alpha shape mesh based on a provided point cloud and alpha parameter and save it as an obj file 
    
    Parameters
    ------
    pointCloud : 2d-array
        In column, index of the point.  
        In line, array that contains the x, y and z coordinates of the point.
    colors : 2d-array
        In column, index of the point.  
        In line, array that contains the r, g and b values of the point (between 0 and 255).
    alpha : float
        Value dictating the level of detail of the result of the alpha shape algorithm 
    path : str
        Path to save the obj file

    """
    minZ = np.PINF
    maxZ = np.NINF

    # Get the min an max of z coordinates
    for point in pointCloud:
        if point[2] < minZ:
            minZ = point[2]
        if point[2] > maxZ:
            maxZ = point[2]

    start = time.time()
    meshCreation.log.info("Starting creation of 2D alpha shape")

    alphashapeTree = alphashape([x[:-1] for x in pointCloud], alpha)

    end = time.time()
    meshCreation.log.info("Finished creation of 2D alpha shape in " + str(end - start) + " seconds") # time in seconds

    start = time.time()
    meshCreation.log.info("Starting triangulation of 2D alpha shape")

    triangles = triangulate.triangulate(alphashapeTree.exterior.coords[:-1])

    index = 1
    vertices = {}
    obj = ""

    # Creation of the obj file
    for tri in triangles:
        for vertex in tri:
            if vertex not in vertices:

                color = [0.46,0.49,0.39]
                """
                for point in pointCloud:
                    if vertex[0]==point[0] and vertex[1]==point[1]:
                        color = colors[pointToColor(point)]
                        break    
                """
                        
                # Adding the top points
                obj += "v " + "{:.8f}".format(vertex[0]) + " " + "{:.8f}".format(vertex[1]) + " " + "{:.8f}".format(maxZ) + " "
                obj += "{:.8f}".format(color[0]) + " " + "{:.8f}".format(color[1]) + " " + "{:.8f}".format(color[2]) + "\n"
                vertices[vertex] = index
                index += 1


    for coord in vertices.keys():

        color = [0.46,0.49,0.39]
        """
        for point in pointCloud:
                    if coord[0]==point[0] and coord[1]==point[1]:
                        color = colors[pointToColor(point)]
                        break
        """
                        
        # Adding the bottom points 
        # Note : it was done this way to have the bottom point at the top point index + the number of point in total at the top
        # e.g. The top point is stored at the index 2 and there are 10 points, the bottom corresponding point is stored at the index 12
        obj += "v " + "{:.8f}".format(coord[0]) + " " + "{:.8f}".format(coord[1]) + " " + "{:.8f}".format(minZ) + " "
        obj += "{:.8f}".format(color[0]) + " " + "{:.8f}".format(color[1]) + " " + "{:.8f}".format(color[2]) + "\n"

    
    # Adding the top flat part
    for tri in triangles:
        obj += "f"
        for vertex in tri:
            obj += " " + str(vertices[vertex])
        obj += "\n" 


    length = len(vertices.keys())

    # Adding the side triangles
    for i in vertices.values():
        if i == length:
            obj += "f " + str(i) + " " + str(i + length) + " " + str(length+1) + "\n"
            obj += "f " + str(i) + " " + str(length+1) + " " + str(1) + "\n"
        else:
            obj += "f " + str(i) + " " + str(i + length) + " " + str((i + 1) + length) + "\n"
            obj += "f " + str(i) + " " + str((i + 1) + length) + " " + str((i + 1)) + "\n"


    with open(path, 'w') as f:
            for line in obj:
                f.write(line)
    
    end = time.time()
    meshCreation.log.info("Finished triangulation of 2D alpha shape in " + str(end - start) + " seconds") # time in seconds


def createAlphashape(pointCloud, alpha, colors, path):
    """Create an alpha shape mesh based on a provided point cloud and alpha parameter and save it as an obj file 
    
    Parameters
    ------
    pointCloud : 2d-array
        In column, index of the point.  
        In line, array that contains the x, y and z coordinates of the point.
    colors : 2d-array
        In column, index of the point.  
        In line, array that contains the r, g and b values of the point (between 0 and 255).
    alpha : float
        Value dictating the level of detail of the result of the alpha shape algorithm 
    path : str
        Path to save the obj file

    """
    start = time.time()
    meshCreation.log.info("Starting creation of alpha shape")
    alphashapeTree = alphashape(pointCloud, alpha)


    end = time.time()
    meshCreation.log.info("Finished creation of alpha shape in " + str(end - start) + " seconds") # time in seconds
    
    # Doesn't seem to affect to mesh much but isn't costly
    alphashapeTree.fill_holes()

    end = time.time()
    
    
    start = time.time()
    meshCreation.log.info("Starting repairing alpha shape's normals")

    alphashapeTree = repairAlphaShapeNormals(alphashapeTree)

    end = time.time()
    meshCreation.log.info("Finished reparing alpha shape's normals in " + str(end - start) + " seconds") # time in seconds

    start = time.time()
    meshCreation.log.info("Starting coloring alpha shape")
    # Reconnecting the color to the vertices
    pointColors = []
    for vertice in alphashapeTree.vertices:
        pointColors.append(colors[pointToColor(vertice)])
    alphashapeTree.visual.vertex_colors = pointColors
    end = time.time()
    meshCreation.log.info("Finished coloring alpha shape in " + str(end - start) + " seconds") # time in seconds

    alphashapeTree.export(path)
    
    return


def repairAlphaShapeNormals(mesh: trimesh.Trimesh):
    """Make all normals face away from the center of the bounding box of the mesh 
    
    Parameters
    ------
    mesh : Trimesh
        The mesh with normals to work with (using face normals) 

    """
    boundingCenter = (mesh.bounds[0]+mesh.bounds[1])/2

    for index in range(mesh.faces.shape[0]):
        faceIndexes = mesh.faces[index]
        face = [mesh.vertices[faceIndexes[0]], mesh.vertices[faceIndexes[1]], mesh.vertices[faceIndexes[2]]]
        normal = mesh.face_normals[index]
        triangleCenter = (face[0] + face[1] + face[2])/3
        if needToReverseFace(boundingCenter, triangleCenter, normal):
            # Flipping the normal
            mesh.faces[index] = np.ascontiguousarray(np.flipud(mesh.faces[index]))

    return mesh


def needToReverseFace(boundingCenter, triangleCenter, normal):
    """Given the center of the bounding box, the center of the triangle and the normal, returns whether the normal needs to be flipped
    
    Parameters
    ------
    boundingCenter : array
        The coordinates (x, y, z) of the bounding box
    triangleCenter : array
        The coordinates (x, y, z) of the triangle center
    normal : array
        The values (x, y, z) of the normal
    
    Returns
    ------
    needToBeReversed : bool
        True if the normal is pointing towards the center of the bounding box
    """
    direction = triangleCenter - boundingCenter
    direction /= scipy.linalg.norm(direction)

    res = np.dot(direction, normal)

    if res>=0:
        return False
    else:
        return True
    

