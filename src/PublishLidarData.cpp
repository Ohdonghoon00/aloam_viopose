#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <unistd.h>
#include <stdio.h>
#include <algorithm>
#include <chrono>
#include <sstream>
#include <glog/logging.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include "common.h"



ros::Publisher pubLaserCloud;

using namespace std;

// Extrinsic parameter rig - lidar
const Eigen::Matrix4d LidarToRig = To44RT(lidar2rig_pose);

LidarData lidar_data;
DataBase DB;

int ReadVIOdata(const std::string Path, DataBase *db)
{
    std::ifstream Rovins2PoseFile(Path, std::ifstream::in);

    if(!Rovins2PoseFile.is_open()){
        std::cout << " Rovins2Pose file failed to open " << std::endl;
        return EXIT_FAILURE;
    }

    std::string line;
    int line_num = 0;

    while(std::getline(Rovins2PoseFile, line) && ros::ok()){

        std::string value;
        std::vector<std::string> values;        

        // values[0]        -> camera fidx
        // values[1] ~ [6]  -> lidar pose (vio result)
        
        std::stringstream ss(line);
        while(std::getline(ss, value, ' '))
            values.push_back(value);

        Vector6d pos;
        pos << std::stod(values[1]), std::stod(values[2]), std::stod(values[3]), std::stod(values[4]), std::stod(values[5]), std::stod(values[6]);


        db->VIOLidarPoses.push_back(pos);

        line_num++;
    }

    Rovins2PoseFile.close();
    return EXIT_SUCCESS;
}

// int ReadSlamdata(const std::string Path, DataBase* db)
// {
//     std::ifstream SlamPoseFile(Path, std::ifstream::in);

//     if(!SlamPoseFile.is_open()){
//         std::cout << " SlamPoseFile failed to open " << std::endl;
//         return EXIT_FAILURE;        
//     }

//     std::string line;
//     int line_num = 0;

//     while(std::getline(SlamPoseFile, line) && ros::ok()){

//         std::string value;
//         std::vector<std::string> values;

//         // values[0]        -> camera fidx
//         // values[1] ~ [6]  -> slam rig? pose
//         // values[7]        -> timestamps

//         std::stringstream ss(line);
//         while(std::getline(ss, value, ' '))
//             values.push_back(value);

//         Vector6d pos;
//         pos << std::stod(values[1]), std::stod(values[2]), std::stod(values[3]), std::stod(values[4]), std::stod(values[5]), std::stod(values[6]);
//         Eigen::Matrix4d Rigpos = To44RT(pos);
//         Eigen::Matrix4d Lidarpos = LidarToRig.inverse() * Rigpos * LidarToRig;

//         db->SlamKFtimestamps.push_back(std::stod(values[7]) * 10e-10 );
//         db->SlamKFPoses.push_back(To6DOF(Lidarpos));
//         db->Slamcamidxs.push_back(std::stoi(values[0]));

//         line_num++;        
//     }
//     SlamPoseFile.close();
//     return EXIT_SUCCESS;
// }

int ReadLidardata(const std::string Path, const std::string LidarBinaryPath, DataBase* db)
{
    std::ifstream LidarcsvFile(Path, std::ifstream::in);

    if(!LidarcsvFile.is_open()){
        std::cout << " Lidar csv file failed to open " << std::endl;
        return EXIT_FAILURE;
    }
    
    // Read Lidar timestamp.csv
    Eigen::Matrix4d LidarRotation;
    std::string Lidarcsvline;
    int Lidarline_num(1);

    while(std::getline(LidarcsvFile, Lidarcsvline) && ros::ok())
    {
        if(Lidarline_num == 1){
            Lidarline_num++;
            continue;
        }
        std::string value;
        std::vector<std::string> values;
        
        // values[0] -> First Seq Timestamp (ns)
        // values[1] -> Last Seq Timestamp (ns)
        // values[2] -> Fidx
        // values[3] -> Num pts
        // values[4] -> Date
        
        std::stringstream ss(Lidarcsvline);
        while(std::getline(ss, value, ','))
            values.push_back(value);
        int fidx = std::stoi(values[2]);
        
        // Binary Data Path
        std::stringstream Lidar_binary_path;
        Lidar_binary_path <<    LidarBinaryPath << std::setfill('0') << 
                                std::setw(5) << fidx << ".bin";
        
        std::ifstream ifs(Lidar_binary_path.str(), std::ifstream::in);
        
        if (!ifs.is_open()){
            std::cout << "bin file failed to open: " << std::endl;
            return EXIT_FAILURE;
        }        
                
        // Read Binary data file
        int PointNum = 0;
        ifs.read((char*) &PointNum, sizeof(int));
        
        float Points[PointNum * 3];
        ifs.read((char*) &Points, sizeof(float) * PointNum * 3);
        
        Eigen::Matrix3Xd Points_(3, PointNum);
        for(int i = 0; i < PointNum; i++){
            Points_(0, i) = static_cast<double>(Points[3 * i]);
            Points_(1, i) = static_cast<double>(Points[3 * i + 1]);
            Points_(2, i) = static_cast<double>(Points[3 * i + 2]);
        }
        
        db->LidarPoints.push_back(Points_);

        ifs.close();
        Lidarline_num++;
    }    
    
    LidarcsvFile.close();
    return EXIT_SUCCESS;
}


int main(int argc, char **argv) 
{
    
    ros::init(argc, argv, "PublishData");
    ros::NodeHandle nh("~");
    
    std::string data_dir;
    int publish_delay;
    
    nh.getParam("data_dir", data_dir);
    nh.getParam("publish_delay", publish_delay);
    
    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 2);

    ros::Publisher pubVIOodometry = nh.advertise<nav_msgs::Odometry>("/VIO_odom_to_init", 100);
    ros::Publisher pubVIOPath = nh.advertise<nav_msgs::Path>("/VIO_odom_path", 100);
    nav_msgs::Path VIOPath;
    
    
    ///////////// VIO pose data /////////////
    std::cout << " Load VIO Data ... " << std::endl;
    std::string VIOPoses_lidarframes = data_dir + "VIOPoses_lidarframes.txt";

    ReadVIOdata(VIOPoses_lidarframes, &DB);

    //////////// Undistortion Points ////////////////
    std::cout << " Load Lidar Data ... " << std::endl;
    std::string LidarcsvPath = data_dir + "lidar_timestamp.csv";
    std::string Lidar_binary_path = data_dir + "lidar2/";
    ReadLidardata(LidarcsvPath, Lidar_binary_path, &DB);

    std::cout << std::endl;
    std::cout << " Finish !! " << std::endl;
    std::cout << " Start Publish !!! " << std::endl;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////// Publish LidarScanTimeStamp ///////////////////////////////////////////////////

    // publish delay
    ros::Rate r(10.0);


    int Publish_cnt(0), frame_cnt(0);
    for(size_t i = 0; i < DB.LidarPoints.size(); i++){
        
        std::vector<Eigen::Vector3d> PublishPoints;
        PublishPoints.clear();

        for(int j = 0; j < DB.LidarPoints[i].cols(); j++){

            Eigen::Vector3d point;
            point.x() = DB.LidarPoints[i](0, j);
            point.y() = DB.LidarPoints[i](1, j);
            point.z() = DB.LidarPoints[i](2, j);
            PublishPoints.push_back(point);
        } 

        Eigen::Quaterniond q = ToQuaternion(DB.VIOLidarPoses[i]);
        Eigen::Vector3d p;
        p << DB.VIOLidarPoses[i][3], DB.VIOLidarPoses[i][4], DB.VIOLidarPoses[i][5];            

        

        if(frame_cnt % 10 == 0){
        std::cout << "Publish !!  Publish num is : " << Publish_cnt << std::endl;

        // publish pointcloud
        sensor_msgs::PointCloud2 output;
        output = ConverToROSmsg(PublishPoints);
        output.header.stamp = ros::Time::now();
        output.header.frame_id = "/camera_init";
        pubLaserCloud.publish(output);


        // publish odometry
        nav_msgs::Odometry VIOodometry;
        VIOodometry.header.frame_id = LidarFrame;
        VIOodometry.child_frame_id = "/laser_odom";
        VIOodometry.header.stamp = output.header.stamp;
        VIOodometry.pose.pose.orientation.x = q.x();
        VIOodometry.pose.pose.orientation.y = q.y();
        VIOodometry.pose.pose.orientation.z = q.z();
        VIOodometry.pose.pose.orientation.w = q.w();
        VIOodometry.pose.pose.position.x = p.x();
        VIOodometry.pose.pose.position.y = p.y();
        VIOodometry.pose.pose.position.z = p.z();
        pubVIOodometry.publish(VIOodometry);

        geometry_msgs::PoseStamped VIOPose;
        VIOPose.header = VIOodometry.header;
        VIOPose.pose = VIOodometry.pose.pose;
        VIOPath.header.stamp = VIOodometry.header.stamp;
        VIOPath.poses.push_back(VIOPose);
        VIOPath.header.frame_id = LidarFrame;
        pubVIOPath.publish(VIOPath);

        Publish_cnt++;
        }
        
        frame_cnt++;
        if(!ros::ok()) break;
        r.sleep();
    }

    std::cout << "Total Publish num is : " << Publish_cnt << std::endl;   
    
    return 0;
}


    

    

    
    






       
            
    

       
            

            


    







             

    



