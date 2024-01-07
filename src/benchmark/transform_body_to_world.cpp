/**
 * @file transform_body_to_world.cpp
 * @author Linfu Wei (ghowoght@qq.com)
 * @brief 将点云从机体坐标系转换到世界坐标系 
 * @version 1.0
 * @date 2023-12-05
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <filesystem>
#include <string>
#include <vector>
#include <fstream>
#include <algorithm>

// 创建带时间的点云类型
struct PointXYZIT{
    PCL_ADD_POINT4D; // 添加pcl里面的XYZ
    float intensity; // 点云强度
    double time; // 点云时间
    int label; // 点云标签
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 16字节对齐
} EIGEN_ALIGN16; // 16字节对齐
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIT, // 注册点云类型
    (float, x, x) // x
    (float, y, y) // y
    (float, z, z) // z
    (float, intensity, intensity) // intensity
    (double, time, time) // time
    (int, label, label) // label
)

using PointType = PointXYZIT;
using PointCloud = pcl::PointCloud<PointType>;

int main(int argc, char ** argv){
    ros::init(argc, argv, "transform_body_to_world");
    ros::NodeHandle nh;

    std::string path = "";
    ros::param::get("~path", path);
    if(path == ""){
        std::cout << "please input path" << std::endl;
        return -1;
    }
    std::cout << "path: " << path << std::endl;

    // 读取文件夹下所有文件夹名,并存入vector    
    std::vector<std::string> sub_paths;
    for (const auto & entry : std::filesystem::directory_iterator(path)){
        sub_paths.push_back(entry.path());
    }
    std::sort(sub_paths.begin(), sub_paths.end());
    for(auto & sub_path : sub_paths){
        std::cout << sub_path << std::endl;
    }

    auto sub_path = sub_paths.back();

    auto hba_path = sub_path + "/hba_output";
    auto pcd_path = hba_path + "/pcd";
    auto pcd_out_path = hba_path + "/pcd_world_balm";
    if(!std::filesystem::exists(pcd_out_path)){
        std::filesystem::create_directory(pcd_out_path);
    }
    auto pose_file = hba_path + "/pose_with_time_balm.txt";
    auto pcd_total_file = hba_path + "/pcd_total_balm.pcd";
    // auto pose_with_time_file = hba_path + "/pose_with_time_balm2.txt";
    // std::ofstream pose_with_time_ofs(pose_with_time_file);
    // 读取位姿
    std::vector<Eigen::Matrix3d> Rs;
    std::vector<Eigen::Vector3d> ts;
    std::vector<double> times;
    std::ifstream pose_ifs(pose_file);
    if(!pose_ifs.is_open()){
        std::cout << "open pose file failed" << std::endl;
        return -1;
    }
    while(!pose_ifs.eof()){
        Eigen::Vector3d t;
        Eigen::Quaterniond q;
        double time;
        pose_ifs >> time;
        pose_ifs >> t[0] >> t[1] >> t[2];
        pose_ifs >> q.x() >> q.y() >> q.z() >> q.w();
        Eigen::Matrix3d R = q.toRotationMatrix();
        Rs.push_back(R);
        ts.push_back(t);
    }
    pose_ifs.close();
    std::cout << "pose size: " << Rs.size() << std::endl;

    // 读取点云
    std::vector<std::string> pcd_files;
    for (const auto & entry : std::filesystem::directory_iterator(pcd_path)){
        pcd_files.push_back(entry.path());
    }

    std::sort(pcd_files.begin(), pcd_files.end(), [](const std::string & a, const std::string & b){
        return std::stoi(a.substr(a.find_last_of('/') + 1, a.find_last_of('.') - a.find_last_of('/') - 1)) < std::stoi(b.substr(b.find_last_of('/') + 1, b.find_last_of('.') - b.find_last_of('/') - 1));
    });

    PointCloud::Ptr cloud_total(new PointCloud);
    for(int i = 0; i < pcd_files.size(); ++i){
        std::cout << "pcd file: " << pcd_files[i] << std::endl;
        PointCloud::Ptr cloud(new PointCloud);
        PointCloud::Ptr cloud_world(new PointCloud);
        if(pcl::io::loadPCDFile<PointType>(pcd_files[i], *cloud) == -1){
            std::cout << "load pcd file failed" << std::endl;
            return -1;
        }
        for(int j = 0; j < cloud->size(); ++j){
            Eigen::Vector3d p(cloud->points[j].x, cloud->points[j].y, cloud->points[j].z);
            p = Rs[i] * p + ts[i];
            PointType p_world = cloud->points[j];
            p_world.x = p[0];
            p_world.y = p[1];
            p_world.z = p[2];
            p_world.intensity = cloud->points[j].intensity;
            cloud_world->push_back(p_world);
        }
        *cloud_total += *cloud_world;
        pcl::io::savePCDFileBinary(pcd_out_path + "/" + std::to_string(i) + ".pcd", *cloud_world);

        // // 保存位姿和时间
        // const int GPS_LEAP_SECOND  = 18;
        // double unixsecond = times[i];
        // // double unixsecond = cloud->points.front().time + 0.1;
        // double seconds = unixsecond + GPS_LEAP_SECOND - 315964800;

        // double gpsweek    = floor(seconds / 604800);
        // double gpsweeksec = seconds - gpsweek * 604800;
        // pose_with_time_ofs.precision(15);
        // pose_with_time_ofs << gpsweeksec << " ";
        // pose_with_time_ofs << ts[i][0] << " " << ts[i][1] << " " << ts[i][2] << " ";
        // Eigen::Quaterniond q(Rs[i]);
        // pose_with_time_ofs << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " ";
        // pose_with_time_ofs << unixsecond << " ";
        // pose_with_time_ofs << std::endl;
    }
    pcl::io::savePCDFileBinary(pcd_total_file, *cloud_total);
}