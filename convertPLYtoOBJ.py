from os import listdir, remove
from os.path import isfile, join
import meshio
from plyfile import PlyData

path = "./islets_convex/"
path_output = "./islets_convex/obj/"


def convert():
    onlyfiles = [f for f in listdir(path) if isfile(join(path, f))]

    for line in onlyfiles:
        mesh = meshio.read(path+line)
        mesh.write(path_output+line[:-3]+"obj")

    print("Done !")

def convert2():
    """
    Converts the given .ply file to an .obj file
    """
    onlyfiles = [f for f in listdir(path) if isfile(join(path, f))]

    for line in onlyfiles:

        obj_path = path_output+line[:-3]+"obj"
        ply = PlyData.read(path+line)

        with open(obj_path, 'w') as f:
            f.write("# OBJ file\n")

            verteces = ply['vertex']

            for v in verteces:
                p = [v['x'], v['y'], v['z']]
                try:
                    c = [v['red']/256 , v['green']/256 , v['blue']/256 ]
                except ValueError:
                    c = [0, 0, 0]
                a = p + c
                f.write("v %.6f %.6f %.6f %.6f %.6f %.6f \n" % tuple(a))


            if 'face' in ply:
                for i in ply['face']['vertex_indices']:
                    f.write("f")
                    for j in range(i.size):
                        ii = [ i[j]+1 ]
                        f.write(" %d" % tuple(ii) )
                    f.write("\n")


def clearFolders():
    
    onlyfiles = [f for f in listdir(path) if isfile(join(path, f))]
    for file in onlyfiles:
        remove(path+str(file))
    
    onlyfiles = [f for f in listdir(path_output) if isfile(join(path_output, f))]
    for file in onlyfiles:
        remove(path_output+str(file))


convert2()