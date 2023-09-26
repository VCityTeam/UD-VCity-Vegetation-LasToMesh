# Vegetation

## Presentation

The goal of this project is to use open datas (for example the [LIDAR done for the city of Lyon](https://data.grandlyon.com/portail/fr/jeux-de-donnees/nuage-points-lidar-2018-metropole-lyon-format-laz/info)) to extract informations about the vegetation and transform it into multiple meshes that are easily convertible into 3DTiles for an easy visualization in apps like [UD-Viz](https://github.com/VCityTeam/UD-Viz).  
  
In order for the script to work, you'll need a classified and colored point cloud in the laz/las format. The [LIDAR done for the city of Lyon](https://data.grandlyon.com/portail/fr/jeux-de-donnees/nuage-points-lidar-2018-metropole-lyon-format-laz/info) is a good example of data to work with.
A 'cell size' parameter is requiered to split the point cloud into sub-clouds of close enough vegetation. 

The algorithm used to create the mesh vary based on the size of the sub-cloud provided :
* For tiny clouds : we use the [convex hull](https://en.wikipedia.org/wiki/Convex_hull) algorithm
* For really large clouds : we use the [alpha shape](https://en.wikipedia.org/wiki/Alpha_shape) algorithm on a flatenned cloud (2D) and then extrude the result
* For the other clouds : we use the [alpha shape](https://en.wikipedia.org/wiki/Alpha_shape) algorithm. The alpha parameter is determined by the size of the cloud
  
## Technology

We're using Python scripts to process the point clouds.  
The version the script was developped for is **Python 3.9**, any other version is not guaranteed to work.   
We're using some libraries such as [laspy](https://laspy.readthedocs.io/en/latest/), [Open3D](http://www.open3d.org/docs/release/index.html) and [Alpha Shape Toolbox](https://alphashape.readthedocs.io/en/latest/readme.html)

## Install

For Windows :
```bash
git clone https://github.com/VCityTeam/UD-VCity-Vegetation-LasToMesh.git
cd UD-VCity-Vegetation-LasToMesh

python3.9 -m venv venv
. venv/Scripts/activate
pip install -r requirements.txt 
```

## Datas

The input for this program is a classified, colored, las/laz file. The vegetation needs to be classified on at least one of the 3 standard values (3, 4 or 5). The color needs to be in the standard las RGB channel.  
  
Small sample datas are included but you can download large scale datas [here](https://data.grandlyon.com/portail/fr/jeux-de-donnees/nuage-points-lidar-2018-metropole-lyon-format-laz/info).

## Parameters

| Command               | Description                                                                                                                 | Default value                                | Example                           |
| --------------------- | --------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------- | --------------------------------- |
|  `-i` / `--input`     | The input las file. For the moment only a single file can be treated at a time.                                             | `./SampleDatas/ExampleDataIsolatedTrees.las` | `-i ./ExampleData.las`            |
|  `-o` / `--output`    | The output folder. Created if non existing. <br> :warning: **The content of the folder will be deleted if not empty !**     | `./output/`                                  | `-o ./outputFolder/`              |
|  `-c` / `--cellsize`  | The size each cell must take inside the grid. The grid is used to split the point cloud into sub-clouds of near vegetation. | `2.0`                                        | `-c 1.5`                          |
|  `-v` / `--verbose`   | Increase the output verbosity.                                                                                              | None                                         | `-v` <br>(no additional argument) |



## Usage

Refer to the 'Install' section for the installation commands.  
Download datas, for example [here](https://data.grandlyon.com/portail/fr/jeux-de-donnees/nuage-points-lidar-2018-metropole-lyon-format-laz/info).    
  
For the following command, we're assuming you downloaded a file nammed ExampleData.las and that it is located in the same folder as the script.

```bash
# if not already in venv
. venv/Scripts/activate

python mainCLI.py -i ExampleData.las -o .\outputFolder\ -c 2.0
```

Note : the output folder does not need to exist beforehand, but if it does, all existing files will be wiped out. So be careful. It is therefore not advised to use `.\` as an argument ;)  

What you'll get is a folder filled with obj files. You then can use 3D-model viewer to visualize your results or else use [py3dtilers](https://github.com/VCityTeam/py3dtilers) and more specifically the
[obj-tiler](https://github.com/VCityTeam/py3dtilers/tree/master/py3dtilers/ObjTiler#obj-tiler) to transform the meshes into 3D tiles.  
Finally, consider using [UD-Viz](https://github.com/VCityTeam/UD-Viz) to view the 3D-tiles inside a web app.
