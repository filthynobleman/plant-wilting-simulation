# Plant Wilting Simulation
This repository contains the implementation of the simulator for the plant wilting process, as described in...  

The repository is a modification of the [Position Based Dynamics solver](https://github.com/InteractiveComputerGraphics/PositionBasedDynamics),
and contains as external dependency the [water diffusion model](https://github.com/filthynobleman/plant-water-diffusion), as described in the
publication.


## Building Instructions
The building process is entirely based on Git and CMake.  
The water diffusion model must be build on its own:
```sh
	cd extern/pwd-repo
	mkdir build
	cd build
	cmake .. -DBUILD_SAMPLES=OFF -DCMAKE_INSTALL_PREFIX="../.."
	cmake --build . --config release -j
	cmake --install .
```
This will produce the water diffusion library and will install it in the directory `extern/pwd/`.  

After that, the application can be built safely from the project root directory:
```sh
	mkdir build
	cd build
	cmake ..
	cmake --build . --config release
```
This will produce an executable `PlantDemo` inside the directory `bin`.


## Running Instructions
The executable `PlantDemo` can be run without arguments, which will make it load the default plant.  
It is possible to specify the plant as the unique extra argument:
```sh
	PlantDemo /path/to/plant.txt
```
Some test plant files can be found in the directory `bin/resources/plants/`.


## Plant File Format
Besides the test plants in the directory `bin/resources/plants/`, it is possible to define a custom plant.  
A file describing a plant is defined according to the following format
```
verts num_verts
id,tail_x,tail_y,tail_z,radius,on_leaf[,is_fixed]
...
edges num_edges
id1,id2
...
```
The first line specifies the number of vertices `num_verts` after the keyword `verts`. The actual nodes are specified in the
following `num_verts` lines.  
In each line, the node ID is specified with `id`. Each node represents a segment of the plant, discretized as a cylinder. The tail
of the node is specified by the coordinated `tail_x,tail_y,tail_z`, whereas the radius of the cylinder is given by `radius`.  
Each node is supposed to have its head in the same point as the tail of its parent node. The root node is supposed to have its head
at the origin `0,0,0`. The root node is determined automatically as the node closer to the origin.  
The `on_leaf` value is always either `0` or `1`, and determines if the node is part of the venation of a leaf or if its
part of a branch.  
The valua `is_fixed` is always either `0` or `1` and it's optional (by default, `0`). It determines if that node is kept in place 
by external supports, and this feature of the simulation can be enabled and disabled at runtime.  
The number of edges `num_edges` is specified after the keyword `edges`. If the plant is a unique connected component, then
`num_edges = num_verts - 1`.  
Each edge is encoded as a couple of node IDs, separated by a comma.