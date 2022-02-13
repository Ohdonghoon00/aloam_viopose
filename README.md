# aloam_viopose

## Dependency

- Ubuntu 64bit 16.04 or 18.04
- ROS kinetic or melodic
    - rosbag
    - ros node handler
    - msg
    - subscribe, publish
- Ceres
- PCL
- Eigen
- Boost? → timer??

## Build

```
mkdir -p catkin/src
cd catkin/src
git clone ~~~~
cd ../..
catkin_make --only-pkg-with-deps aloam
source devel/setup.bash
```

## Run

```
roscore
roslaunch aloam aloam.launch
roslaunch aloam TestPublishData.launch
```

## Data경로 수정
