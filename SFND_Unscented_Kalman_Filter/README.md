# Unscented Kalman Filter

## Dependencies

### Ubuntu

One can build and install `VTK` and `PCL` from source. This is what I did.

```shell script
# Prerequisite
sudo apt install build-essential libgl1-mesa-dev libglu1-mesa-dev
sudo apt install libflann-dev libboost-all-dev libeigen3-dev 

# Build and install VTK
git clone --branch v8.2.0 https://github.com/Kitware/VTK.git
cd VTK
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF ../
make -j4
sudo make install

# Build and install PCL
git clone --branch pcl-1.9.1 https://github.com/PointCloudLibrary/pcl.git
cd pcl
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_visualization=ON ../
make -j4
sudo make install

# Build and install Eigen
git clone --branch 3.3.7 https://github.com/eigenteam/eigen-git-mirror.git
cd eigen-git-mirror
mkdir build && cd build
cmake .. && make install
```

## Build and Run

```shell script
cd SFND_Lidar_Obstacle_Detection
mkdir build && cd build
cmake .. && make
./environment ../data/pcd/data_1
```

## Algorithms

Please check [here](../UKF).
