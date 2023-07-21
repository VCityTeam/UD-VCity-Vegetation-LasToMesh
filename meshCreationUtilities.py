from os import listdir, remove, mkdir
from os.path import isfile, join

def pointToColor(point):
    return str(point[0])+str(point[1])+str(point[2])

def clearFolders(path):
    path_output = path+"obj/"

    try:
        onlyfiles = [f for f in listdir(path) if isfile(join(path, f))]
        for file in onlyfiles:
            remove(path+str(file))
    except FileNotFoundError:
        mkdir(path)
    
    try:
        onlyfiles = [f for f in listdir(path_output) if isfile(join(path_output, f))]
        for file in onlyfiles:
            remove(path_output+str(file))
    except FileNotFoundError:
        # No obj folder existing
        mkdir(path_output)
        return
    