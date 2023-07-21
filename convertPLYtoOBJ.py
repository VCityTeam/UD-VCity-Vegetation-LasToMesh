from os import listdir, remove
from os.path import isfile, join
from plyfile import PlyData



def convert2(path):
    """
    Converts the given .ply file to an .obj file
    """
    path_output = path + "obj/"
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



path = "./islets_convex/"

convert2(path)