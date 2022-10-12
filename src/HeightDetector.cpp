#include "liquid_height_estimation/HeightDetector.h"

HeightDetector::HeightDetector(ros::NodeHandle& nh)
{
    this->nh_ = nh;
    this->initializeSubscribers();
    this->initializePublishers();

    input_cloud = pclPointer(new pclCloud);
    cloud_transformed = pclPointer(new pclCloud);

    ros::Time now = ros::Time::now();
    listener.waitForTransform("zed_left_camera_frame", "camera_fixed", ros::Time(0), ros::Duration(3.0));
    listener.lookupTransform("zed_left_camera_frame", "camera_fixed", ros::Time(0), transform);

    eigen_tf = Eigen::Matrix4f::Identity();
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            eigen_tf(i, j) = transform.getBasis()[i][j];
        }
        eigen_tf(i, 3) = transform.getOrigin()[i];
    }
    eigen_tf(3, 3) = 0.0;
        
    std::cout << eigen_tf << "\n";
    std::cout << "Height Detector is constructed" << "\n";
}

void HeightDetector::initializeSubscribers()
{
    this->cloud_sub = this->nh_.subscribe("/height_detector/input_cloud", 1, &HeightDetector::cloud_cb, this);
    std::cout << "Subscribers are initialized" << "\n";
}

void HeightDetector::initializePublishers()
{
    this->height_pub = this->nh_.advertise<std_msgs::Float64>("/height_detector/height", 1);
    this->test_cloud_pub = this->nh_.advertise<sensor_msgs::PointCloud2>("/height_detector/test_cloud", 1);
    
    std::cout << "Publishers are initialized" << "\n";
}

HeightDetector::~HeightDetector()
{
    std::cout << "Height Detector is destructed" << "\n";
}

pclCloud HeightDetector::removeNaNs(pcl::PassThrough<PointT> pass,
                                    pclPointer& in_cloud)
{
    std::cout << "Removing NaNs..." << std::endl;
    pclPointer out_cloud(new pclCloud);
    pass.setInputCloud(in_cloud);
    pass.filter(*out_cloud);
    return *out_cloud;
}

void HeightDetector::clearClouds()
{
    this->input_cloud->clear();
    this->input_cloud.reset(new pclCloud);

    std::cout << "Cleared clouds" << "\n";
}

void HeightDetector::cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input_cloud)
{
    std::cout << "Received cloud message" << "\n";
    fixed_cloud = *input_cloud;
    fixed_cloud.header.frame_id = "camera_fixed";
    fixed_cloud.header.stamp = ros::Time::now();

    pcl::fromROSMsg(fixed_cloud, *(this->input_cloud));
    *(this->input_cloud) = this->removeNaNs(this->pass, this->input_cloud);
    pcl::transformPointCloud(*(this->input_cloud), *cloud_transformed, eigen_tf);

    this->clearClouds();
}