import laspy
import numpy as np
import math
import time
import open3d as o3d
from alphashape import alphashape
import meshCreationUtilities as MeshUtilities
import argparse

def main():
    startProg = time.time()

    parser = argparse.ArgumentParser(description="Transform the vegetation inside a classified point cloud into a set of meshes")

    parser.add_argument("-i", "--input", help="Input las file (default ='./SampleDatas/ExampleDataIsolatedTrees.las')", default="./SampleDatas/ExampleDataIsolatedTrees.las")
    parser.add_argument("-o", "--output", help="Output directory (default ='./output/')", default="./output/")
    parser.add_argument("-c", "--cellsize", help="Cell size (default = 2.0)", default=2.0, type=float)
    parser.add_argument("-v", "--verbose", help="Increase output verbosity", action="store_true")

    args = parser.parse_args()
    input_path = args.input
    output_path = args.output

    MeshUtilities.clearFolders(output_path)

    # To replace with CLI
    las = laspy.read(input_path)

    xmax=-math.inf
    xmin=math.inf
    ymax=-math.inf
    ymin=math.inf

    # Change the value to get a more precise representation
    cellSize = args.cellsize

    # Used to filter the point cloud (needs to be classified) 

    pcFiltered = laspy.create(point_format=las.header.point_format, file_version=las.header.version)
    pcFiltered.points = las.points[(las.classification == 3) | (las.classification == 4) | (las.classification == 5)]

    # When the point cloud is already filtered
    # pcFiltered = las


    point_data = np.stack([pcFiltered.x, pcFiltered.y, pcFiltered.z], axis=0).transpose((1, 0))
    # RGB colors which values vary between 0 and 255
    point_data_color = np.stack([pcFiltered.red, pcFiltered.green, pcFiltered.blue], axis=0).transpose((1, 0))
    # If the color is coded on 16 bits, force it back to 8 bits
    if point_data_color[0][0] > 256:
        point_data_color = point_data_color//256
    # RGB colors which values vary between 0 and 1
    point_data_color_normalized = point_data_color/255

    xmax, ymax, zmax = np.max(point_data, axis=0)
    xmin, ymin, zmin = np.min(point_data, axis=0)


    boxWidth = xmax-xmin
    boxHeight = ymax - ymin

    cellCountWidth = math.ceil(boxWidth/cellSize)
    cellCountHeight = math.ceil(boxHeight/cellSize)

    # Init
    cellsZmean = np.zeros((cellCountWidth, cellCountHeight))
    cellsZCount = np.zeros((cellCountWidth, cellCountHeight))

    end = time.time()
    print("Finished initialisation in " + str(end - startProg) + " seconds") # time in seconds


    start = time.time()
    print("Starting init of cellZMean")

    indexes = np.ndarray(point_data.shape, np.int32)

    # Used to convert spacial coordinates to tab index
    indexes[:,0] =  np.floor(((point_data[:,0]-xmin)/cellSize)-1)
    indexes[:,1] =  np.floor(((point_data[:,1]-ymin)/cellSize)-1)


    for i in range(indexes.shape[0]):
        xi = indexes[i,0]
        yi = indexes[i,1]

        # Z max instead of mean (comment out the next two lines if you want to use Zmax)
        # cellsZmean[xi][yi] = max(cellsZmean[xi][yi], point[2])

        # Sum of the height in one cell
        cellsZmean[xi][yi] += point_data[i,2]
        cellsZCount[xi][yi] += 1



    for i in range(cellsZmean.shape[0]):
        for j in range(cellsZmean.shape[1]):
            if(cellsZCount[i][j] > 0):
                # Calculate the mean of height in one cell
                cellsZmean[i][j] = cellsZmean[i][j]/cellsZCount[i][j]

    end = time.time()
    print("Finished init of cellZMean in " + str(end - start) + " seconds") # time in seconds


    start = time.time()
    print("Starting process of vertices")

    # This variable is used for the implementation of canopy reconstruction. It is no longer used so it has been commented out
    #
    # Creation of the obj file through a string
    # objfile = "# Vegetation.obj \n# \n\n"

    cellsVertexIndex = np.zeros((cellCountWidth, cellCountHeight), np.int32)

    index = 1

    # Warning : going through the tab by col and not by row 
    for i in range(cellsZmean.shape[0]):
        for j in range(cellsZmean.shape[1]):
            if(cellsZmean[i][j] > 0.0):
                xcoord = i * cellSize + xmin 
                ycoord = j * cellSize + ymin
                # objfile += "v " + str(xcoord) + " " + str(ycoord) + " " +  str(cellsZmean[i][j]) + "\n"
                cellsVertexIndex[i][j] = index
                index += 1

    # objfile += "\n"

    end = time.time()
    print("Finished process of vertices in " + str(end - start) + " seconds") # time in seconds


    start = time.time()
    print("Starting process of triangles")

    cellsIsletIndex = np.zeros((cellCountWidth, cellCountHeight), np.int32)
    index_islet = 1

    # Currently, this part is only used to calculate the islet ids the parts used to calculate the canopy have been commented out
    #
    # Checking the current cell and the other three on a given corner (here curentx +1 and current y+1)
    # If at least 3 of them have a height, draw a triangle using those cells' center as vertices
    # If the four of them have a height, draw two triangles instead
    for i in range(cellsZmean.shape[0]-1):
        for j in range(cellsZmean.shape[1]-1):
            if (cellsZmean[i][j] > 0 and cellsZmean[i+1][j] > 0 and cellsZmean[i+1][j+1] > 0 and cellsZmean[i][j+1] > 0):
                # Double triangle
                
                # First triangle


                # objfile += "f "
                # objfile += str(cellsVertexIndex[i][j]) + " "
                # objfile += str(cellsVertexIndex[i+1][j]) + " "
                # objfile += str(cellsVertexIndex[i][j+1]) + "\n"

                # Second triangle


                # objfile += "f "
                # objfile += str(cellsVertexIndex[i][j+1]) + " "
                # objfile += str(cellsVertexIndex[i+1][j]) + " "
                # objfile += str(cellsVertexIndex[i+1][j+1]) + "\n"

                # Indexing the islet number

                # If the cell already has a tag, we spread it to the other cells
                if(cellsIsletIndex[i][j] != 0):
                    # If the cell we try to tag already has a tag we delete the other one
                    # and replace it with the new one (on all the existing cells with that tag)
                    if(cellsIsletIndex[i+1][j] != 0 and cellsIsletIndex[i+1][j] != cellsIsletIndex[i][j]):
                        cellsIsletIndex[cellsIsletIndex==cellsIsletIndex[i+1][j]] = cellsIsletIndex[i][j]
                    else:
                        cellsIsletIndex[i+1][j] = cellsIsletIndex[i][j]

                    if(cellsIsletIndex[i][j+1] != 0 and cellsIsletIndex[i][j+1] != cellsIsletIndex[i][j]):
                        cellsIsletIndex[cellsIsletIndex==cellsIsletIndex[i][j+1]] = cellsIsletIndex[i][j]
                    else:
                        cellsIsletIndex[i][j+1] = cellsIsletIndex[i][j]

                    if(cellsIsletIndex[i+1][j+1] != 0 and cellsIsletIndex[i+1][j+1] != cellsIsletIndex[i][j]):
                        cellsIsletIndex[cellsIsletIndex==cellsIsletIndex[i+1][j+1]] = cellsIsletIndex[i][j]
                    else:
                        cellsIsletIndex[i+1][j+1] = cellsIsletIndex[i][j]


                elif(cellsIsletIndex[i+1][j] != 0):

                    if(cellsIsletIndex[i][j] != 0 and cellsIsletIndex[i][j] != cellsIsletIndex[i+1][j]):
                        cellsIsletIndex[cellsIsletIndex==cellsIsletIndex[i][j]] = cellsIsletIndex[i+1][j]
                    else:
                        cellsIsletIndex[i][j] = cellsIsletIndex[i+1][j]

                    if(cellsIsletIndex[i][j+1] != 0 and cellsIsletIndex[i][j+1] != cellsIsletIndex[i+1][j]):
                        cellsIsletIndex[cellsIsletIndex==cellsIsletIndex[i][j+1]] = cellsIsletIndex[i+1][j]
                    else:
                        cellsIsletIndex[i][j+1] = cellsIsletIndex[i+1][j]

                    if(cellsIsletIndex[i+1][j+1] != 0 and cellsIsletIndex[i+1][j+1] != cellsIsletIndex[i+1][j]):
                        cellsIsletIndex[cellsIsletIndex==cellsIsletIndex[i+1][j+1]] = cellsIsletIndex[i+1][j]
                    else:
                        cellsIsletIndex[i+1][j+1] = cellsIsletIndex[i+1][j]
                    
                elif(cellsIsletIndex[i][j+1] != 0):
                    
                    if(cellsIsletIndex[i+1][j] != 0 and cellsIsletIndex[i+1][j] != cellsIsletIndex[i][j+1]):
                        cellsIsletIndex[cellsIsletIndex==cellsIsletIndex[i+1][j]] = cellsIsletIndex[i][j+1]
                    else:
                        cellsIsletIndex[i+1][j] = cellsIsletIndex[i][j+1]

                    if(cellsIsletIndex[i][j] != 0 and cellsIsletIndex[i][j] != cellsIsletIndex[i][j+1]):
                        cellsIsletIndex[cellsIsletIndex==cellsIsletIndex[i][j]] = cellsIsletIndex[i][j+1]
                    else:
                        cellsIsletIndex[i][j] = cellsIsletIndex[i][j+1]

                    if(cellsIsletIndex[i+1][j+1] != 0 and cellsIsletIndex[i+1][j+1] != cellsIsletIndex[i][j+1]):
                        cellsIsletIndex[cellsIsletIndex==cellsIsletIndex[i+1][j+1]] = cellsIsletIndex[i][j+1]
                    else:
                        cellsIsletIndex[i+1][j+1] = cellsIsletIndex[i][j+1]
                
                elif(cellsIsletIndex[i+1][j+1] != 0):
                    
                    if(cellsIsletIndex[i+1][j] != 0 and cellsIsletIndex[i+1][j] != cellsIsletIndex[i+1][j+1]):
                        cellsIsletIndex[cellsIsletIndex==cellsIsletIndex[i+1][j]] = cellsIsletIndex[i+1][j+1]
                    else:
                        cellsIsletIndex[i+1][j] = cellsIsletIndex[i+1][j+1]

                    if(cellsIsletIndex[i][j+1] != 0 and cellsIsletIndex[i][j+1] != cellsIsletIndex[i+1][j+1]):
                        cellsIsletIndex[cellsIsletIndex==cellsIsletIndex[i][j+1]] = cellsIsletIndex[i+1][j+1]
                    else:
                        cellsIsletIndex[i][j+1] = cellsIsletIndex[i+1][j+1]

                    if(cellsIsletIndex[i][j] != 0 and cellsIsletIndex[i][j] != cellsIsletIndex[i+1][j+1]):
                        cellsIsletIndex[cellsIsletIndex==cellsIsletIndex[i][j]] = cellsIsletIndex[i+1][j+1]
                    else:
                        cellsIsletIndex[i][j] = cellsIsletIndex[i+1][j+1]
                
                # Else, if none of the cells already have a tag we give them the next tag available in the list    
                else :
                    cellsIsletIndex[i][j] = index_islet
                    cellsIsletIndex[i+1][j] = index_islet
                    cellsIsletIndex[i][j+1] = index_islet
                    cellsIsletIndex[i+1][j+1] = index_islet
                    index_islet += 1

            elif ((cellsZmean[i][j]) > 0 and (cellsZmean[i+1][j]) > 0 and (cellsZmean[i+1][j+1]) > 0 and (cellsZmean[i][j+1]) == 0):
                # Main angle in the bottom right corner
                

                # objfile += "f "
                # objfile += str(cellsVertexIndex[i][j]) + " "
                # objfile += str(cellsVertexIndex[i+1][j]) + " "
                # objfile += str(cellsVertexIndex[i+1][j+1]) + "\n"

                # Indexing the islet number

                # If the cell already has a tag, we spread it to the other cells
                if(cellsIsletIndex[i][j] != 0):

                    # If the cell we try to tag already has a tag we delete the other one
                    # and replace it with the new one (on all the existing cells with that tag)
                    if(cellsIsletIndex[i+1][j] != 0 and cellsIsletIndex[i+1][j] != cellsIsletIndex[i][j]):
                        cellsIsletIndex[cellsIsletIndex==cellsIsletIndex[i+1][j]] = cellsIsletIndex[i][j]
                    else:
                        cellsIsletIndex[i+1][j] = cellsIsletIndex[i][j]

                    if(cellsIsletIndex[i+1][j+1] != 0 and cellsIsletIndex[i+1][j+1] != cellsIsletIndex[i][j]):
                        cellsIsletIndex[cellsIsletIndex==cellsIsletIndex[i+1][j+1]] = cellsIsletIndex[i][j]
                    else:
                        cellsIsletIndex[i+1][j+1] = cellsIsletIndex[i][j]

                elif(cellsIsletIndex[i+1][j] != 0):

                    if(cellsIsletIndex[i][j] != 0 and cellsIsletIndex[i][j] != cellsIsletIndex[i+1][j]):
                        cellsIsletIndex[cellsIsletIndex==cellsIsletIndex[i][j]] = cellsIsletIndex[i+1][j]
                    else:
                        cellsIsletIndex[i][j] = cellsIsletIndex[i+1][j]

                    if(cellsIsletIndex[i+1][j+1] != 0 and cellsIsletIndex[i+1][j+1] != cellsIsletIndex[i+1][j]):
                        cellsIsletIndex[cellsIsletIndex==cellsIsletIndex[i+1][j+1]] = cellsIsletIndex[i+1][j]
                    else:
                        cellsIsletIndex[i+1][j+1] = cellsIsletIndex[i+1][j]
                    
                elif(cellsIsletIndex[i+1][j+1] != 0):

                    if(cellsIsletIndex[i+1][j] != 0 and cellsIsletIndex[i+1][j] != cellsIsletIndex[i+1][j+1]):
                        cellsIsletIndex[cellsIsletIndex==cellsIsletIndex[i+1][j]] = cellsIsletIndex[i+1][j+1]
                    else:
                        cellsIsletIndex[i+1][j] = cellsIsletIndex[i+1][j+1]

                    if(cellsIsletIndex[i+1][j+1] != 0 and cellsIsletIndex[i+1][j+1] != cellsIsletIndex[i+1][j+1]):
                        cellsIsletIndex[cellsIsletIndex==cellsIsletIndex[i+1][j+1]] = cellsIsletIndex[i+1][j+1]
                    else:
                        cellsIsletIndex[i+1][j+1] = cellsIsletIndex[i+1][j+1]

                # Else, if none of the cells already have a tag we give them the next tag available in the list    
                else :
                    cellsIsletIndex[i][j] = index_islet
                    cellsIsletIndex[i+1][j] = index_islet
                    cellsIsletIndex[i+1][j+1] = index_islet
                    index_islet += 1


                
                

            elif ((cellsZmean[i][j]) > 0 and (cellsZmean[i+1][j]) > 0 and (cellsZmean[i+1][j+1]) == 0 and (cellsZmean[i][j+1]) > 0):
                # Main angle in the bottom left corner
                
                # objfile += "f "
                # objfile += str(cellsVertexIndex[i][j]) + " "
                # objfile += str(cellsVertexIndex[i+1][j]) + " "
                # objfile += str(cellsVertexIndex[i][j+1]) + "\n" 

                # Indexing the islet number

                # If the cell already has a tag, we spread it to the other cells
                if(cellsIsletIndex[i][j] != 0):

                    # If the cell we try to tag already has a tag we delete the other one
                    # and replace it with the new one (on all the existing cells with that tag)
                    if(cellsIsletIndex[i+1][j] != 0 and cellsIsletIndex[i+1][j] != cellsIsletIndex[i][j]):
                        cellsIsletIndex[cellsIsletIndex==cellsIsletIndex[i+1][j]] = cellsIsletIndex[i][j]
                    else:
                        cellsIsletIndex[i+1][j] = cellsIsletIndex[i][j]

                    if(cellsIsletIndex[i][j+1] != 0 and cellsIsletIndex[i][j+1] != cellsIsletIndex[i][j]):
                        cellsIsletIndex[cellsIsletIndex==cellsIsletIndex[i][j+1]] = cellsIsletIndex[i][j]
                    else:
                        cellsIsletIndex[i][j+1] = cellsIsletIndex[i][j]

                elif(cellsIsletIndex[i+1][j] != 0):
                    
                    if(cellsIsletIndex[i][j] != 0 and cellsIsletIndex[i][j] != cellsIsletIndex[i+1][j]):
                        cellsIsletIndex[cellsIsletIndex==cellsIsletIndex[i][j]] = cellsIsletIndex[i+1][j]
                    else:
                        cellsIsletIndex[i][j] = cellsIsletIndex[i+1][j]

                    if(cellsIsletIndex[i][j+1] != 0 and cellsIsletIndex[i][j+1] != cellsIsletIndex[i+1][j]):
                        cellsIsletIndex[cellsIsletIndex==cellsIsletIndex[i][j+1]] = cellsIsletIndex[i+1][j]
                    else:
                        cellsIsletIndex[i][j+1] = cellsIsletIndex[i+1][j]
                    
                elif(cellsIsletIndex[i][j+1] != 0):

                    if(cellsIsletIndex[i+1][j] != 0 and cellsIsletIndex[i+1][j] != cellsIsletIndex[i][j+1]):
                        cellsIsletIndex[cellsIsletIndex==cellsIsletIndex[i+1][j]] = cellsIsletIndex[i][j+1]
                    else:
                        cellsIsletIndex[i+1][j] = cellsIsletIndex[i][j+1]

                    if(cellsIsletIndex[i][j] != 0 and cellsIsletIndex[i][j] != cellsIsletIndex[i][j+1]):
                        cellsIsletIndex[cellsIsletIndex==cellsIsletIndex[i][j]] = cellsIsletIndex[i][j+1]
                    else:
                        cellsIsletIndex[i][j] = cellsIsletIndex[i][j+1]


                    cellsIsletIndex[i][j] = cellsIsletIndex[i][j+1]
                    cellsIsletIndex[i+1][j] = cellsIsletIndex[i][j+1]

                # Else, if none of the cells already have a tag we give them the next tag available in the list    
                else :
                    cellsIsletIndex[i][j] = index_islet
                    cellsIsletIndex[i+1][j] = index_islet
                    cellsIsletIndex[i][j+1] = index_islet
                    index_islet += 1


                

            elif ((cellsZmean[i][j]) > 0 and (cellsZmean[i+1][j]) == 0 and (cellsZmean[i+1][j+1]) > 0 and (cellsZmean[i][j+1]) > 0):
                # Main angle in the top left corner


                # objfile += "f "
                # objfile += str(cellsVertexIndex[i][j]) + " "
                # objfile += str(cellsVertexIndex[i+1][j+1]) + " "
                # objfile += str(cellsVertexIndex[i][j+1]) + "\n"

                # Indexing the islet number

                # If the cell already has a tag, we spread it to the other cells
                if(cellsIsletIndex[i][j] != 0):

                    # If the cell we try to tag already has a tag we delete the other one
                    # and replace it with the new one (on all the existing cells with that tag)
                    if(cellsIsletIndex[i][j+1] != 0 and cellsIsletIndex[i][j+1] != cellsIsletIndex[i][j]):
                        cellsIsletIndex[cellsIsletIndex==cellsIsletIndex[i][j+1]] = cellsIsletIndex[i][j]
                    else:
                        cellsIsletIndex[i][j+1] = cellsIsletIndex[i][j]

                    if(cellsIsletIndex[i+1][j+1] != 0 and cellsIsletIndex[i+1][j+1] != cellsIsletIndex[i][j]):
                        cellsIsletIndex[cellsIsletIndex==cellsIsletIndex[i+1][j+1]] = cellsIsletIndex[i][j]
                    else:
                        cellsIsletIndex[i+1][j+1] = cellsIsletIndex[i][j]

                elif(cellsIsletIndex[i][j+1] != 0):

                    if(cellsIsletIndex[i][j] != 0 and cellsIsletIndex[i][j] != cellsIsletIndex[i][j+1]):
                        cellsIsletIndex[cellsIsletIndex==cellsIsletIndex[i][j]] = cellsIsletIndex[i][j+1]
                    else:
                        cellsIsletIndex[i][j] = cellsIsletIndex[i][j+1]

                    if(cellsIsletIndex[i+1][j+1] != 0 and cellsIsletIndex[i+1][j+1] != cellsIsletIndex[i][j+1]):
                        cellsIsletIndex[cellsIsletIndex==cellsIsletIndex[i+1][j+1]] = cellsIsletIndex[i][j+1]
                    else:
                        cellsIsletIndex[i+1][j+1] = cellsIsletIndex[i][j+1]
                
                elif(cellsIsletIndex[i+1][j+1] != 0):

                    if(cellsIsletIndex[i][j+1] != 0 and cellsIsletIndex[i][j+1] != cellsIsletIndex[i+1][j+1]):
                        cellsIsletIndex[cellsIsletIndex==cellsIsletIndex[i][j+1]] = cellsIsletIndex[i+1][j+1]
                    else:
                        cellsIsletIndex[i][j+1] = cellsIsletIndex[i+1][j+1]

                    if(cellsIsletIndex[i][j] != 0 and cellsIsletIndex[i][j] != cellsIsletIndex[i+1][j+1]):
                        cellsIsletIndex[cellsIsletIndex==cellsIsletIndex[i][j]] = cellsIsletIndex[i+1][j+1]
                    else:
                        cellsIsletIndex[i][j] = cellsIsletIndex[i+1][j+1]

                # Else, if none of the cells already have a tag we give them the next tag available in the list
                else :
                    cellsIsletIndex[i][j] = index_islet
                    cellsIsletIndex[i][j+1] = index_islet
                    cellsIsletIndex[i+1][j+1] = index_islet
                    index_islet += 1

                
            elif ((cellsZmean[i][j]) == 0 and (cellsZmean[i+1][j]) > 0 and (cellsZmean[i+1][j+1]) > 0 and (cellsZmean[i][j+1]) > 0):
                # Main angle in the top right corner
                

                # objfile += "f "
                # objfile += str(cellsVertexIndex[i][j+1]) + " "
                # objfile += str(cellsVertexIndex[i+1][j]) + " "
                # objfile += str(cellsVertexIndex[i+1][j+1]) + "\n"

                # Indexing the islet number

                # If the cell already has a tag, we spread it to the other cells
                if(cellsIsletIndex[i+1][j] != 0):

                    # If the cell we try to tag already has a tag we delete the other one
                    # and replace it with the new one (on all the existing cells with that tag)
                    if(cellsIsletIndex[i][j+1] != 0 and cellsIsletIndex[i][j+1] != cellsIsletIndex[i+1][j]):
                        cellsIsletIndex[cellsIsletIndex==cellsIsletIndex[i][j+1]] = cellsIsletIndex[i+1][j]
                    else:
                        cellsIsletIndex[i][j+1] = cellsIsletIndex[i+1][j]

                    if(cellsIsletIndex[i+1][j+1] != 0 and cellsIsletIndex[i+1][j+1] != cellsIsletIndex[i+1][j]):
                        cellsIsletIndex[cellsIsletIndex==cellsIsletIndex[i+1][j+1]] = cellsIsletIndex[i+1][j]
                    else:
                        cellsIsletIndex[i+1][j+1] = cellsIsletIndex[i+1][j]
                    
                elif(cellsIsletIndex[i][j+1] != 0):

                    if(cellsIsletIndex[i+1][j] != 0 and cellsIsletIndex[i+1][j] != cellsIsletIndex[i][j+1]):
                        cellsIsletIndex[cellsIsletIndex==cellsIsletIndex[i+1][j]] = cellsIsletIndex[i][j+1]
                    else:
                        cellsIsletIndex[i+1][j] = cellsIsletIndex[i][j+1]

                    if(cellsIsletIndex[i+1][j+1] != 0 and cellsIsletIndex[i+1][j+1] != cellsIsletIndex[i][j+1]):
                        cellsIsletIndex[cellsIsletIndex==cellsIsletIndex[i+1][j+1]] = cellsIsletIndex[i][j+1]
                    else:
                        cellsIsletIndex[i+1][j+1] = cellsIsletIndex[i][j+1]
                
                elif(cellsIsletIndex[i+1][j+1] != 0):

                    if(cellsIsletIndex[i+1][j] != 0 and cellsIsletIndex[i+1][j] != cellsIsletIndex[i+1][j+1]):
                        cellsIsletIndex[cellsIsletIndex==cellsIsletIndex[i+1][j]] = cellsIsletIndex[i+1][j+1]
                    else:
                        cellsIsletIndex[i+1][j] = cellsIsletIndex[i+1][j+1]

                    if(cellsIsletIndex[i][j+1] != 0 and cellsIsletIndex[i][j+1] != cellsIsletIndex[i+1][j+1]):
                        cellsIsletIndex[cellsIsletIndex==cellsIsletIndex[i][j+1]] = cellsIsletIndex[i+1][j+1]
                    else:
                        cellsIsletIndex[i][j+1] = cellsIsletIndex[i+1][j+1]


                    cellsIsletIndex[i+1][j] = cellsIsletIndex[i+1][j+1]
                    cellsIsletIndex[i][j+1] = cellsIsletIndex[i+1][j+1]
            
            # Else, if none of the cells already have a tag we give them the next tag available in the list
                else :
                    cellsIsletIndex[i+1][j] = index_islet
                    cellsIsletIndex[i][j+1] = index_islet
                    cellsIsletIndex[i+1][j+1] = index_islet
                    index_islet += 1
            

    end = time.time()
    print("Finished process of triangles in " + str(end - start) + " seconds") # time in seconds

    start = time.time()
    print("Starting separating islets")

    # Separate islands of points based on the island id calculated above
    # and put the informations in different structures

    # Dict containing in keys the id of the island and in values the list of points of the island (x, y, z) 
    islands = {}
    # Used a bit bellow, contains in keys the island's id and in value its size (based on the cellsize)
    islands_size = {}
    # Dict containing the color associated with the point
    # The key is obtained by passing the coordinates to the 'pointToColor' function
    # The value is the RGB value of the point, ranging from 0 to 255
    islands_color = {}
    # Dict containing the normalized color associated with the point
    # The key is obtained by passing the coordinates to the 'pointToColor' function
    # The value is the RGB value of the point, ranging from 0 to 1
    islands_color_normalized = {}

    for i in range(indexes.shape[0]):
        xi = indexes[i,0]+1
        yi = indexes[i,1]+1

        if cellsIsletIndex[xi][yi] != 0:
            
            if cellsIsletIndex[xi][yi] not in islands:
                islands[cellsIsletIndex[xi][yi]] = []
    
            islands[cellsIsletIndex[xi][yi]].append(point_data[i])

            islands_color[MeshUtilities.pointToColor(point_data[i])] = point_data_color[i]
            islands_color_normalized[MeshUtilities.pointToColor(point_data[i])] = point_data_color_normalized[i]
            


    end = time.time()
    print("Finished separating islets in " + str(end - start) + " seconds") # time in seconds

    startoutput = time.time()
    print("Starting output of islets convex hull/alpha shapes")

    # For debuging purposes
    choice = ""

    for index in islands:

        condition = cellsIsletIndex == index
        # Count the number of cells the island is spanning 
        islands_size[index] = np.count_nonzero(condition)
        
        # If the island is too small, don't create a mesh for it
        if(len(islands[index]) > 8):
            # If the island has an area of less than 50 cells, the mesh will be a convex hull, else it'll be an alpha shape 
            if(islands_size[index] < 50):
                try:
                    print("convex hull")
                    
                    path = output_path+"hull_"+ str(index) +".obj"

                    MeshUtilities.createConvexHull(islands[index], islands_color_normalized, path)

                    choice += str(islands_size[index]) + " convex hull\n"

                except Exception as exce:
                    # Sometimes, error happens due to the shape of the point cloud
                    # we use the alpha shape algorithm instead
                    print("[Warning] : error when creating the convex hull, aborting")
                    print(exce)
                    choice += str(islands_size[index]) + " convex hull ERROR\n"
            # If the island is too big, create an extruded 2D alpha shape
            elif(islands_size[index] > 10000):
                # Alpha parameter
                alpha = MeshUtilities.computeAlpha(islands[index])
                
                path = output_path+"alpha_extruded_"+ str(index) + ".obj"
                        
                MeshUtilities.createExtruded2DAlphaShape(islands[index], islands_color_normalized, alpha, path)
            # If it's not too big or too small, try doing a layered alpha shape
            else:
                try:

                    # Divide the island in layers based on the z axis
                    foo, foo, zmax = np.max(islands[index], axis=0)
                    foo, foo, zmin = np.min(islands[index], axis=0)
                    
                    # number of slices we want from our island
                    nbLayers = 5
                    # We take a portion of the upper slice to smooth the transition between the meshes
                    margin = ((zmax-zmin)/nbLayers)/3

                    for layer in range(1,(nbLayers+1)):
                        # Compute the bounding heights 
                        thresholdmax = zmin + (((zmax-zmin)/nbLayers)*layer)
                        thresholdmin = zmin + (((zmax-zmin)/(nbLayers))*(layer-1))
                        layerPoints = []
                        for point in islands[index]:
                            if(point[2] > thresholdmin and point[2] < thresholdmax + margin):
                                layerPoints.append(point)

                        # Not enough points in the layer
                        if len(layerPoints) < 6:
                            continue
                        
                        alpha = MeshUtilities.computeAlpha(islands[index])

                        path = output_path+"alpha_"+ str(index) + "_" + str(layer) + ".obj"
                        
                        MeshUtilities.createAlphashape(layerPoints, alpha, islands_color, path)
                    
                    choice += str(islands_size[index]) + " sliced alpha shape\n"
                except Exception as exce:
                    # Sometimes, error happens due to the shape of the point cloud (needs to be confirmed)
                    print("[Warning] : error when creating the layered alphashape, aborting")
                    print(exce)
                    try:
                        # Simple alpha shape

                        alpha = MeshUtilities.computeAlpha(islands[index])
                        path = output_path+"alpha_"+ str(index) + ".obj"

                        MeshUtilities.createAlphashape(islands[index], alpha, islands_color, path)

                        choice += str(islands_size[index]) + " sliced alpha shape ERROR alpha shape OK\n"
                    except Exception as exce:
                        try:
                            print("[Warning] : error when creating the alphashape, aborting")
                            print(exce)
                            # If even the simple alpha shape failed, try creating a convex hull

                            path = output_path+"hull_"+ str(index) +".obj"

                            MeshUtilities.createConvexHull(islands[index], islands_color_normalized, path)

                            choice += str(islands_size[index]) + " sliced alpha shape ERROR alpha shape ERROR convex hull OK\n"

                        except Exception:
                            return
                        choice += str(islands_size[index]) + " sliced alpha shape ERROR alpha shape ERROR convex hull ERROR\n"

            


    end = time.time()
    print("Finished output of islets convex hull/alpha shapes in " + str(end - startoutput) + " seconds") # time in seconds

    start = time.time()
    print("Starting writing of output file")
    
    # For debug
    #
    # Summary of the size of the island, the script's algorithm choice and if there was an error
    """
    with open('choice.txt', 'w') as f:
        for line in choice:
            f.write(line)
    """
            
    # Export for the canopy algorithm
    #
    # Write all the lines in the obj file
    """
    with open('output.obj', 'w') as f:
        for line in objfile:
            f.write(line)
    """

    end = time.time()
    print("Finished writing of output file in " + str(end - start) + " seconds") # time in seconds
    print("Finished execution in " + str(end - startProg) + " seconds") # time in seconds


if __name__ == "__main__":
    main()