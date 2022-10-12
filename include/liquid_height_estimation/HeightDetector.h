#pragma once

// ROS related libraries
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>

// Other libraries
#include <signal.h>

// PCL related libraries
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> pclCloud;
typedef pcl::PointCloud<PointT>::Ptr pclPointer;

class HeightDetector{
    private:
        ros::NodeHandle nh_;
        ros::Subscriber cloud_sub;
        ros::Publisher height_pub;

        std_msgs::Float64 height_msg;

        pclPointer input_cloud;

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