from os import listdir
from os.path import isfile, join
import meshio

path = "./islets_convex/"
path_output = "./islets_convex/obj/"

onlyfiles = [f for f in listdir(path) if isfile(join(path, f))]

for line in onlyfiles:
    mesh = meshio.read(path+line)
    mesh.write(path_output+line[:-3]+"obj")

print("Done !")