#include "liquid_height_estimation/HeightDetector.h"

HeightDetector::HeightDetector(ros::NodeHandle& nh)
{
    this->nh_ = nh;
    this->initializeSubscribers();
    this->initializePublishers();
    f = boost::bind(&HeightDetector::reconfigureCB, this, _1, _2);
    server.setCallback(f);

    input_cloud = pclPointer(new pclCloud);
    cloud_transformed = pclPointer(new pclCloud);
    cloud_scene = pclPointer(new pclCloud);

    field_cond = pcl::ConditionAnd<PointT>::Ptr(new pcl::ConditionAnd<PointT>);

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

pclCloud HeightDetector::removeFields(pcl::ConditionAnd<PointT>::Ptr& range_cond,
                                        pclPointer& in_cloud)
{
    std::cout << "Removing fields..." << std::endl;
    pclPointer out_cloud(new pclCloud);
    condrem.setCondition(range_cond);
    condrem.setInputCloud(in_cloud);
    condrem.setKeepOrganized(true);
    condrem.filter(*out_cloud);
    return *out_cloud;
}

void HeightDetector::reconfigureCB(liquid_height_estimation::HeightDetectorConfig& config, uint32_t level)
{
    ROS_INFO("Reconfigure Request");
    this->min_x = config.min_x;
    this->max_x = config.max_x;
    this->min_y = config.min_y;
    this->max_y = config.max_y;
    this->min_z = config.min_z;
    this->max_z = config.max_z;
}

void HeightDetector::cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input_cloud)
{
    std::cout << "Received cloud message" << "\n";
    fixed_cloud = *input_cloud;
    fixed_cloud.header.frame_id = "zed_left_camera_frame";
    fixed_cloud.header.stamp = ros::Time::now();

    pcl::fromROSMsg(fixed_cloud, *(this->input_cloud));
    *(this->input_cloud) = this->removeNaNs(this->pass, this->input_cloud);
    pcl::transformPointCloud(*(this->input_cloud), *cloud_transformed, eigen_tf);
    *cloud_transformed = this->removeNaNs(this->pass, cloud_transformed);
    
    field_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::GT, min_x)));
    field_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::LT, max_x)));
    field_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::GT, min_y)));
    field_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::LT, max_y)));
    field_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::GT, min_z)));
    field_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::LT, max_z)));
    
    *cloud_scene = this->removeFields(field_cond, cloud_transformed);

    pcl::toROSMsg(*cloud_scene, test_cloud);
    test_cloud.header.frame_id = "zed_left_camera_frame";
    test_cloud.header.stamp = input_cloud->header.stamp;
    this->test_cloud_pub.publish(test_cloud);
    
    this->clearClouds();
}