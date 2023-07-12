# Vegetation

This repo is linked to JPE's internship, read more in the [projet's page](https://github.com/VCityTeam/VCity/blob/master/Projects/Gamagora/VegetationInternship.md)

## Goal

The goal of this project is to use open datas (for example the [LIDAR done for the city of Lyon](https://data.grandlyon.com/jeux-de-donnees/nuage-points-lidar-2018-metropole-lyon-format-laz/info)) to extract informations about the vegetation and transform it into a format easily convertible into 3DTile for an easy visualization in apps like [UD-Viz](https://github.com/VCityTeam/UD-Viz).  
  
We wanted to make a mesh representation of the vegetation and since we're using point clouds as inputs we need to use algorithms to convert these point clouds into meshes.  
There are many methods that can be used, the ones we chose are the [convex hull](https://en.wikipedia.org/wiki/Convex_hull) and the [alpha shape](https://en.wikipedia.org/wiki/Alpha_shape).  
  
We will probably use more methods to define multiple LODs.

## Technology
 
We're using Python scripts to process the point clouds. We're using some libraries such as [laspy](https://laspy.readthedocs.io/en/latest/), [Open3D](http://www.open3d.org/docs/release/index.html), [Alpha Shape Toolbox](https://alphashape.readthedocs.io/en/latest/readme.html) and [plyfile](https://github.com/dranjan/python-plyfile)
