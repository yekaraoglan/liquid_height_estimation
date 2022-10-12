#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>

#include <signal.h>

class HeightDetector{
    private:
        ros::NodeHandle nh_;
        ros::Subscriber cloud_sub;
        ros::Publisher height_pub;

        std_msgs::Float64 height_msg;

        void cloud_cb(const sensor_msgs::PointCloud2ConstPtr&);
        void initializePublishers();
        void initializeSubscribers();

    public:
        HeightDetector(ros::NodeHandle&);
        ~HeightDetector();
};