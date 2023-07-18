# Vegetation

## Goal

The goal of this project is to use open datas (for example the [LIDAR done for the city of Lyon](https://data.grandlyon.com/jeux-de-donnees/nuage-points-lidar-2018-metropole-lyon-format-laz/info)) to extract informations about the vegetation and transform it into a format easily convertible into 3DTile for an easy visualization in apps like [UD-Viz](https://github.com/VCityTeam/UD-Viz).  
  
We wanted to make a mesh representation of the vegetation and since we're using point clouds as inputs we need to use algorithms to convert these point clouds into meshes.  
There are many methods that can be used, the ones we chose are the [convex hull](https://en.wikipedia.org/wiki/Convex_hull) and the [alpha shape](https://en.wikipedia.org/wiki/Alpha_shape).  
  
We will probably use more methods to define multiple LODs.

## Technology
 
We're using Python scripts to process the point clouds. We're using some libraries such as [laspy](https://laspy.readthedocs.io/en/latest/), [Open3D](http://www.open3d.org/docs/release/index.html), [Alpha Shape Toolbox](https://alphashape.readthedocs.io/en/latest/readme.html) and [plyfile](https://github.com/dranjan/python-plyfile)

## Install

```bash
git clone https://github.com/VCityTeam/UD-VCity-Vegetation-LasToMesh.git
cd UD-VCity-Vegetation-LasToMesh

python -m venv venv
source venv/bin/activate
pip install -r requirements.txt 
```
## Datas

The input for this program is a classified, colored, las file. The vegetation needs to be classified on at least one of the 3 standard values (3, 4 or 5). The color needs to be in the standard las RGB channel.  
  
You can download large scale datas [here](https://data.grandlyon.com/jeux-de-donnees/nuage-points-lidar-2018-metropole-lyon-format-laz/info). Be careful, you'll often find laz files, the compressed format of las file. You'll need to use tools like lazip to get to the las format.
