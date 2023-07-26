from os import listdir, remove, mkdir
from os.path import isfile, join

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
