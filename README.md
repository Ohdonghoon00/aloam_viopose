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
- Boost → timer??

## Build

```
mkdir -p catkin/src
cd catkin/src
git clone https://github.com/Ohdonghoon00/aloam_viopose.git
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

## Data Structure Ex
```
 DATA_DIR
    ├─ lidar_timestamp.csv // lidar timestamp and fidx			
    ├─ lidar2	// Undistortion lidar points
    	├─ 00000.bin
    	├─ 00001.bin
    	└─ ...						
    └─ VIOPoses_lidarframes.txt // VIO pose
```

## Output File
```
Result_DIR
    ├─ aloam_mapping_highfrequency_pose.txt ( fast )
    └─ aloam_mapping_pose.txt // LaserMapping Pose Result ( slow but exact)
```

## Data 경로 수정

**DATA_DIR**
- launch 폴더내 PublishData.launch 파일 data_dir value값 수정

**Result_DIR**
- launch 폴더내 aloam.launch 파일 result_dir value
