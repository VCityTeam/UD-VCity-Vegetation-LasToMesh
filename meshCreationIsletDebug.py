import laspy
import numpy as np
import math
import time
import random


startProg = time.time()

las = laspy.read('ExampleData.las')

xmax=-math.inf
xmin=math.inf
ymax=-math.inf
ymin=math.inf

# Change the value to get a more 
cellSize = 1

# Used to filter the point cloud (needs to be classified) 

pcFiltered = laspy.create(point_format=las.header.point_format, file_version=las.header.version)
pcFiltered.points = las.points[las.classification == 5]

# When the point cloud is already filtered
# pcFiltered = las


point_data = np.stack([pcFiltered.x, pcFiltered.y, pcFiltered.z], axis=0).transpose((1, 0))


xmax, ymax, zmax = np.max(point_data, axis=0)
xmin, ymin, zmin = np.min(point_data, axis=0)


boxWidth = xmax-xmin
boxHeight = ymax - ymin

cellCountWidth = math.ceil(boxWidth/cellSize)
cellCountHeight = math.ceil(boxHeight/cellSize)


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

    # Z max au lieu de moyenne (commenter les deux lignes suivantes si utilisation de Zmax)
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

cellsVertexIndex = np.zeros((cellCountWidth, cellCountHeight), np.int32)

index = 1

# Warning : going through the tab by col and not by row 
for i in range(cellsZmean.shape[0]):
    for j in range(cellsZmean.shape[1]):
        if(cellsZmean[i][j] > 0.0):
            cellsVertexIndex[i][j] = index
            index += 1


end = time.time()
print("Finished process of vertices in " + str(end - start) + " seconds") # time in seconds

start = time.time()
print("Starting process of triangles")

cellsIsletIndex = np.zeros((cellCountWidth, cellCountHeight), np.int32)
index_islet = 1

triangles = ""

# Checking the current cell and the other three on a given corner (here curentx +1 and current y+1)
# If at least 3 of them have a height, draw a triangle using those cells' center as vertices
# If the four of them have a height, draw two triangles instead
for i in range(cellsZmean.shape[0]-1):
    for j in range(cellsZmean.shape[1]-1):
        if (cellsZmean[i][j] > 0 and cellsZmean[i+1][j] > 0 and cellsZmean[i+1][j+1] > 0 and cellsZmean[i][j+1] > 0):
            # Double triangle
            
            # First triangle

            # Vertex coloring version
            
            triangles += "f "
            triangles += str(cellsVertexIndex[i][j]) + "//" + str(cellsVertexIndex[i][j]) + " "
            triangles += str(cellsVertexIndex[i+1][j]) + "//" + str(cellsVertexIndex[i+1][j]) + " "
            triangles += str(cellsVertexIndex[i][j+1]) + "//" + str(cellsVertexIndex[i][j+1]) + "\n"

            # objfile += "f "
            # objfile += str(cellsVertexIndex[i][j]) + " "
            # objfile += str(cellsVertexIndex[i+1][j]) + " "
            # objfile += str(cellsVertexIndex[i][j+1]) + "\n"

            # Second triangle

            # Vertex coloring version

            triangles += "f "
            triangles += str(cellsVertexIndex[i][j+1]) + "//" + str(cellsVertexIndex[i][j+1]) + " "
            triangles += str(cellsVertexIndex[i+1][j]) + "//" + str(cellsVertexIndex[i+1][j]) + " "
            triangles += str(cellsVertexIndex[i+1][j+1]) + "//" + str(cellsVertexIndex[i+1][j+1]) + "\n"

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
            
            # Vertex coloring version

            triangles += "f "
            triangles += str(cellsVertexIndex[i][j]) + "//" + str(cellsVertexIndex[i][j]) + " "
            triangles += str(cellsVertexIndex[i+1][j]) + "//" + str(cellsVertexIndex[i+1][j]) + " "
            triangles += str(cellsVertexIndex[i+1][j+1]) + "//" + str(cellsVertexIndex[i+1][j+1]) + "\n"


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
            
            # Vertex coloring version

            triangles += "f "
            triangles += str(cellsVertexIndex[i][j]) + "//" + str(cellsVertexIndex[i][j]) + " "
            triangles += str(cellsVertexIndex[i+1][j]) + "//" + str(cellsVertexIndex[i+1][j]) + " "
            triangles += str(cellsVertexIndex[i][j+1]) + "//" + str(cellsVertexIndex[i][j+1]) + "\n" 
            
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
            
            # Vertex coloring version

            triangles += "f "
            triangles += str(cellsVertexIndex[i][j]) + "//" + str(cellsVertexIndex[i][j]) + " "
            triangles += str(cellsVertexIndex[i+1][j+1]) + "//" + str(cellsVertexIndex[i+1][j+1]) + " "
            triangles += str(cellsVertexIndex[i][j+1]) + "//" + str(cellsVertexIndex[i][j+1]) + "\n"

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
            
            # Vertex coloring version

            triangles += "f "
            triangles += str(cellsVertexIndex[i][j+1]) + "//" + str(cellsVertexIndex[i][j+1]) + " "
            triangles += str(cellsVertexIndex[i+1][j]) + "//" + str(cellsVertexIndex[i+1][j]) + " "
            triangles += str(cellsVertexIndex[i+1][j+1]) + "//" + str(cellsVertexIndex[i+1][j+1]) + "\n"

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
print("Starting process of vertices")

# Creation of the obj file through a string
objfile = "# Vegetation.obj \n# \n\n"

cellsVertexIndex = np.zeros((cellCountWidth, cellCountHeight), np.int32)

index = 1

# Warning : going through the tab by col and not by row 
for i in range(cellsZmean.shape[0]):
    for j in range(cellsZmean.shape[1]):
        if(cellsZmean[i][j] > 0.0):
            xcoord = i * cellSize + xmin 
            ycoord = j * cellSize + ymin
            random.seed(int(cellsIsletIndex[i][j]))

            # The color via vertex color is still not implemented in py3dtilers so it's commented for now
            objfile += "v " + str(xcoord) + " " + str(ycoord) + " " +  str(cellsZmean[i][j]) + " " + str(random.random()) + " " + str(random.random()) + " " + str(random.random()) + "\n"   
            # objfile += "v " + str(xcoord) + " " + str(ycoord) + " " +  str(cellsZmean[i][j]) + "\n"
            cellsVertexIndex[i][j] = index
            index += 1

objfile += "\n"

objfile += triangles

end = time.time()
print("Finished process of vertices in " + str(end - start) + " seconds") # time in seconds



start = time.time()
print("Starting writing of output file")

# Write all the lines in the obj file
with open('output.obj', 'w') as f:
    for line in objfile:
        f.write(line)

with open('outputIslet.txt', 'w') as f:
    for i in range(cellsIsletIndex.shape[0]):
        f.write("\n")
        for j in range(cellsIsletIndex.shape[1]):
            f.write(str(cellsIsletIndex[i][j]) + " ")

end = time.time()
print("Finished writing of output file in " + str(end - start) + " seconds") # time in seconds
print("Finished execution in " + str(end - startProg) + " seconds") # time in seconds