#pragma once

#include <iostream>

// ROS related libraries
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>

// Other libraries
#include <signal.h>

// PCL related libraries
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> pclCloud;
typedef pcl::PointCloud<PointT>::Ptr pclPointer;

class HeightDetector{
    private:
        ros::NodeHandle nh_;
        ros::Subscriber cloud_sub;
        ros::Publisher height_pub;
        ros::Publisher test_cloud_pub;

        tf::StampedTransform transform;
        tf::TransformListener listener;
        tf::Matrix3x3 tf_rot_mat;
        std_msgs::Float64 height_msg;
        sensor_msgs::PointCloud2 fixed_cloud, test_cloud;

        pclPointer input_cloud;
        pclPointer cloud_transformed;
        Eigen::Matrix4f eigen_tf;

        pcl::PassThrough<PointT> pass;

        void cloud_cb(const sensor_msgs::PointCloud2ConstPtr&);
        void initializePublishers();
        void initializeSubscribers();
        void clearClouds(); 
        pclCloud removeNaNs(pcl::PassThrough<PointT>, pclPointer&);
    public:
        HeightDetector(ros::NodeHandle&);
        ~HeightDetector();
};