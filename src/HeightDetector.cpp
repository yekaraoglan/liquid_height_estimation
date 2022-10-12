#include "liquid_height_estimation/HeightDetector.h"

HeightDetector::HeightDetector(ros::NodeHandle& nh)
{
    this->nh_ = nh;
    this->initializeSubscribers();
    this->initializePublishers();

    input_cloud = pclPointer(new pclCloud);
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
    pcl::fromROSMsg(*input_cloud, *(this->input_cloud));
    *(this->input_cloud) = this->removeNaNs(this->pass, this->input_cloud);
    this->clearClouds();
}