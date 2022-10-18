#pragma once

#include <iostream>

// ROS related libraries
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <liquid_height_estimation/HeightDetectorConfig.h>
#include "liquid_height_estimation/Statistics.h"

// Other libraries
#include <signal.h>

// PCL related libraries
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <string>
#include <cmath>
#include <ctime>

#include <Eigen/Geometry>


typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> pclCloud;
typedef pcl::PointCloud<PointT>::Ptr pclPointer;

class HeightDetector{
    private:
        ros::NodeHandle nh_;
        ros::Subscriber cloud_sub;
        ros::Subscriber stats_sub;
        ros::Publisher height_pub;
        ros::Publisher test_cloud_pub;
        ros::Publisher cylinder_coeff_pub;
        double min_x, min_y, min_z, max_x, max_y, max_z;

        double median_x, median_y, median_radius;
        tf::StampedTransform transform;
        tf::TransformListener listener;
        tf::Matrix3x3 tf_rot_mat;
        std_msgs::Float64 height_msg;
        sensor_msgs::PointCloud2 fixed_cloud, test_cloud;
        dynamic_reconfigure::Server<liquid_height_estimation::HeightDetectorConfig> server;
        dynamic_reconfigure::Server<liquid_height_estimation::HeightDetectorConfig>::CallbackType f;

        pclPointer input_cloud;
        pclPointer cloud_transformed;
        pclPointer cloud_scene;
        pclPointer cloud_plane;
        pclPointer cloud_wo_plane;
        pclPointer cloud_filtered;
        pclPointer cloud_inside_of_cup;
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_cylinder;
        Eigen::Matrix4f eigen_tf;

        pcl::PassThrough<PointT> pass;
        pcl::ConditionAnd<PointT>::Ptr field_cond;
        pcl::ConditionalRemoval<PointT> condrem;
        pcl::UniformSampling<PointT> uniform_sampling;
        pcl::NormalEstimation<PointT, pcl::Normal> ne;
        pcl::search::KdTree<PointT>::Ptr tree;
        pcl::PointIndices::Ptr inliers_plane, inliers_cylinder;
        pcl::ModelCoefficients::Ptr coefficients_plane, coefficients_cylinder;
        pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
        pcl::ExtractIndices<PointT> extract;
        pcl::ExtractIndices<pcl::Normal> extract_normals;
        pcl::StatisticalOutlierRemoval<PointT> sor;
        
        void cloud_cb(const sensor_msgs::PointCloud2ConstPtr&);
        void initializePublishers();
        void initializeSubscribers();
        void clearClouds(); 
        pclCloud removeNaNs(pcl::PassThrough<PointT>, pclPointer&);
        void reconfigureCB(liquid_height_estimation::HeightDetectorConfig&, uint32_t);
        pclCloud removeFields(pcl::ConditionAnd<PointT>::Ptr& range_cond, pclPointer& in_cloud);
        pcl::PointCloud<pcl::Normal> estimateNormals(pcl::PointCloud<PointT>::Ptr&);
        void segmentPlane(pclPointer&);
        pclCloud extractPlane(pclPointer&);
        pclCloud removePlane(pclPointer&);
        void segmentCylinder(pclPointer&);
        void statistics_cb(const liquid_height_estimation::Statistics::ConstPtr&);
    public:
        HeightDetector(ros::NodeHandle&);
        ~HeightDetector();
};