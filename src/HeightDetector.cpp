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
    cloud_normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
    cloud_normals_cylinder = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
    cloud_plane = pclPointer(new pclCloud);
    cloud_wo_plane = pclPointer(new pclCloud);
    cloud_filtered = pclPointer(new pclCloud);

    field_cond = pcl::ConditionAnd<PointT>::Ptr(new pcl::ConditionAnd<PointT>);
    tree = pcl::search::KdTree<PointT>::Ptr(new pcl::search::KdTree<PointT> ());
    inliers_plane = pcl::PointIndices::Ptr(new pcl::PointIndices);
    coefficients_plane = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);
    inliers_cylinder = pcl::PointIndices::Ptr(new pcl::PointIndices);
    coefficients_cylinder = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);
   
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

    cloud_transformed->clear();
    cloud_transformed.reset(new pclCloud);

    cloud_scene->clear();
    cloud_scene.reset(new pclCloud);

    cloud_normals->clear();
    cloud_normals.reset(new pcl::PointCloud<pcl::Normal>);

    cloud_plane->clear();
    cloud_plane.reset(new pclCloud);

    cloud_wo_plane->clear();
    cloud_wo_plane.reset(new pclCloud);

    inliers_plane->header.seq = 0;
    inliers_plane->header.stamp = 0;
    inliers_plane->header.frame_id = "";
    inliers_plane->indices = {};

    coefficients_plane->header.seq = 0;
    coefficients_plane->header.stamp = 0;
    coefficients_plane->header.frame_id = "";
    coefficients_plane->values = {};

    inliers_cylinder->header.seq = 0;
    inliers_cylinder->header.stamp = 0;
    inliers_cylinder->header.frame_id = "";
    inliers_cylinder->indices = {};

    coefficients_cylinder->header.seq = 0;
    coefficients_cylinder->header.stamp = 0;
    coefficients_cylinder->header.frame_id = "";
    coefficients_cylinder->values = {};

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

pcl::PointCloud<pcl::Normal> HeightDetector::estimateNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& in_cloud)
{
    std::cout << "Estimating Normals..." << std::endl;
    ne.setSearchMethod(tree);
    ne.setInputCloud(in_cloud);
    ne.setKSearch(25);
    ne.compute(*cloud_normals);
    return *cloud_normals;
}

void HeightDetector::segmentPlane(pclPointer& input_cloud)
{
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight (0.1);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations (30);
    seg.setDistanceThreshold (0.1);
    seg.setInputCloud (input_cloud);
    seg.setInputNormals (cloud_normals);
    seg.segment(*inliers_plane, *coefficients_plane);
}

pclCloud HeightDetector::extractPlane(pclPointer& input_cloud)
{
    extract.setInputCloud(input_cloud);
    extract.setIndices(inliers_plane);
    extract.setNegative(false);
    extract.filter(*cloud_plane);
    return *cloud_plane;
}

pclCloud HeightDetector::removePlane(pclPointer& input_cloud)
{
    pcl::PointCloud<PointT>::Ptr out_cloud(new pcl::PointCloud<PointT>);
    extract.setNegative(true);
    extract.filter(*out_cloud);
    return *out_cloud;
}

void HeightDetector::segmentCylinder(pclPointer& input_cloud)
{
    extract_normals.setNegative(true);
    extract_normals.setInputCloud(cloud_normals);
    extract_normals.setIndices(inliers_plane);
    extract_normals.filter(*cloud_normals_cylinder);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(0.1);
    seg.setMaxIterations(10000);
    seg.setDistanceThreshold(0.05);
    seg.setRadiusLimits(0, 0.2);
    seg.setInputCloud(input_cloud);
    seg.setInputNormals(cloud_normals_cylinder);
    seg.segment(*inliers_cylinder, *coefficients_cylinder);
    
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
    *cloud_scene = this->removeNaNs(this->pass, cloud_scene);

    uniform_sampling.setInputCloud(cloud_scene);
    uniform_sampling.setRadiusSearch(0.001);
    uniform_sampling.filter(*cloud_scene);

    // sor.setInputCloud(cloud_scene);
    // sor.setMeanK(50);
    // sor.setStddevMulThresh(1.0);
    // sor.filter(*cloud_filtered);

    *cloud_normals = this->estimateNormals(cloud_scene);
    this->segmentPlane(cloud_scene);
    *cloud_plane = this->extractPlane(cloud_scene);
    *cloud_wo_plane = this->removePlane(cloud_scene);

    this->segmentCylinder(cloud_wo_plane);

    pcl::toROSMsg(*cloud_wo_plane, test_cloud);
    test_cloud.header.frame_id = "zed_left_camera_frame";
    test_cloud.header.stamp = input_cloud->header.stamp;
    this->test_cloud_pub.publish(test_cloud);
    
    this->clearClouds();
}