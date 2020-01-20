# Travelling-Salesman-Problem
* Follow [link](https://github.com/KrishnaBhatu/Travelling-Salesman-Problem/blob/master/Report.pdf) to the project report
* Following are the instructions to run program
Drectory Structure:

1) src --> It contains the source code
	-mst51.cpp (TSP solved with 2-approximation method and removing intersection heuristics for <eil51.tsp> data file)
	-mst76.cpp (TSP solved with 2-approximation method and removing intersection heuristics for <eil76.tsp> data file)
	-mst101.cpp (TSP solved with 2-approximation method and removing intersection heuristics for <eil101.tsp> data file)
	-random.cpp (TSP solved with 2-approximation method and removing intersection heuristics for randomly generated 100 vertices)
2) input --> It contains the input graph data for TSP (.tsp) files
	-eil51.tsp
	-eil76.tsp
	-eil101.tsp
3) ouput --> It contains the output tour generated 2-approximation method for the given data graph 
	-mst51.output.tour
	-mst76.output.tour
	-mst101.output.tour

Instructions to run file:

Download the entire folder containing the above mentioned sub-directories
Then follow the given instructions;
```
$ cd <path_to_folder>
$ mkdir build
$ cd build
$ cmake -DCMAKE_BUILD_TYPE=Release ..
$ make
```
After this step our source code will be build and compiled, also we will have the executables in the build directory

To execute the TSP for eil51 dataset
```
$ cd <path_to_folder>/build  
$ ./TSP51
```
To execute the TSP for eil76 dataset
```
$ cd <path_to_folder>/build  
$ ./TSP76
```
To execute the TSP for eil101 dataset
```
$ cd <path_to_folder>/build  
$ ./TSP101
```
To execute the TSP for randomly generated dataset
```
$ cd <path_to_folder>/build  
$ ./random
```
