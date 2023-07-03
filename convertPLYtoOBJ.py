from os import listdir, remove
from os.path import isfile, join
import meshio

path = "./islets_convex/"
path_output = "./islets_convex/obj/"
# convert()

def convert():
    onlyfiles = [f for f in listdir(path) if isfile(join(path, f))]

    for line in onlyfiles:
        mesh = meshio.read(path+line)
        mesh.write(path_output+line[:-3]+"obj")

    print("Done !")

def clearFolders():
    
    onlyfiles = [f for f in listdir(path) if isfile(join(path, f))]
    for file in onlyfiles:
        remove(path+str(file))
    
    onlyfiles = [f for f in listdir(path_output) if isfile(join(path_output, f))]
    for file in onlyfiles:
        remove(path_output+str(file))