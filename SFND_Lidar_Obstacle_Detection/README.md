# LIDAR Obstacle Detection


## Dependencies

To install the latest version of [PCL](http://pointclouds.org/) (currently 1.9.1):

#### Ubuntu 

```bash
$ sudo apt install libflann-dev libboost-all-dev libeigen3-dev 

# build and install VTK
$ git clone --branch v8.2.0 https://github.com/Kitware/VTK.git
$ cd VTK
$ mkdir build && cd build
$ cmake -DCMAKE_BUILD_TYPE=Release ../
$ make -j4
$ sudo make install

# build and install PCL
$ git clone --branch pcl-1.9.1 https://github.com/PointCloudLibrary/pcl.git
$ cd pcl
$ mkdir build && cd build
$ cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_visualization=ON ../
$ make -j4
$ sudo make install
```

## Build and Run

```shell script
$ cd SFND_Lidar_Obstacle_Detection
$ mkdir build && cd build
$ cmake ..
$ make
$ ./environment
```

By default, the project uses the in-house implementations for point cloud
segmentation and clustering. To switch to the [PCL](http://pointclouds.org/)
implementation, build the project with:

```shell script
$ cmake .. -DUSE_PCL_SEG=ON -DUSE_PCL_CLUSTER=ON
```
