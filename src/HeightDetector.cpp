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
    cloud_cup = pclPointer(new pclCloud);
    cloud_inside_of_cup = pclPointer(new pclCloud);

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
    median_x = 0.0;
    median_y = 0.0;
    median_radius = 0.0;
    cup_height = 9.8;
    std::cout << eigen_tf << "\n";
    std::cout << "Height Detector is constructed" << "\n";
}

void HeightDetector::initializeSubscribers()
{
    this->cloud_sub = this->nh_.subscribe("/height_detector/input_cloud", 1, &HeightDetector::cloud_cb, this);
    this->stats_sub = this->nh_.subscribe("/height_detector/statistics", 1, &HeightDetector::statistics_cb, this);
    std::cout << "Subscribers are initialized" << "\n";
}

void HeightDetector::initializePublishers()
{
    this->height_pub = this->nh_.advertise<std_msgs::Float64>("/height_detector/height", 1);
    this->test_cloud_pub = this->nh_.advertise<sensor_msgs::PointCloud2>("/height_detector/test_cloud", 1);
    this->cylinder_coeff_pub = this->nh_.advertise<std_msgs::Float64MultiArray>("/height_detector/cylinder_coefficients", 1);
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

    cloud_cup->clear();
    cloud_cup.reset(new pclCloud);

    cloud_inside_of_cup->clear();
    cloud_inside_of_cup.reset(new pclCloud);

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
    pclPointer out_cloud(new pclCloud);
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

    this->min_r = config.min_r;
    this->max_r = config.max_r;
    this->min_g = config.min_g;
    this->max_g = config.max_g;
    this->min_b = config.min_b;
    this->max_b = config.max_b;
}

void HeightDetector::statistics_cb(const liquid_height_estimation::Statistics::ConstPtr& msg)
{
    std::cout << "Statistics received\n";
    this->median_x = msg->medians.data[0];
    this->median_y = msg->medians.data[1];
    this->median_radius = msg->medians.data[3];
}

pclCloud HeightDetector::filterColor(pcl::ConditionAnd<PointT>::Ptr color_cond, 
                                        pclPointer& in_cloud)
{
    std::cout << "Filtering color..." << std::endl;
    pclPointer out_cloud(new pclCloud);
    condrem.setCondition(color_cond);
    condrem.setInputCloud(in_cloud);
    condrem.setKeepOrganized(true);
    condrem.filter(*out_cloud);
    return *out_cloud;    

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

    *cloud_normals = this->estimateNormals(cloud_scene);
    this->segmentPlane(cloud_scene);
    *cloud_plane = this->extractPlane(cloud_scene);
    *cloud_wo_plane = this->removePlane(cloud_scene);

    this->segmentCylinder(cloud_wo_plane);

    std_msgs::Float64MultiArray msg;
    for (auto i: coefficients_cylinder->values)
    {
        msg.data.push_back(i);
    }
    cylinder_coeff_pub.publish(msg);
    msg.data.clear();

    pcl::ConditionAnd<PointT>::Ptr field_cond_cup = pcl::ConditionAnd<PointT>::Ptr(new pcl::ConditionAnd<PointT>);
    
    pcl::FieldComparison<PointT>::ConstPtr y_max = pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::GT, this->median_y - (1.5*median_radius)));
    pcl::FieldComparison<PointT>::ConstPtr y_min = pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::LT, this->median_y + (1.5*median_radius)));
    pcl::FieldComparison<PointT>::ConstPtr x_max = pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::GT, this->median_x - (1.5*median_radius)));
    pcl::FieldComparison<PointT>::ConstPtr x_min = pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::LT, this->median_x + (1.5*median_radius)));
    field_cond_cup->addComparison (y_max); 
    field_cond_cup->addComparison (y_min);
    field_cond_cup->addComparison (x_max);
    field_cond_cup->addComparison (x_min);

    *cloud_cup = this->removeFields(field_cond_cup, cloud_scene);

    pcl::ConditionAnd<PointT>::Ptr color_cond = pcl::ConditionAnd<PointT>::Ptr(new pcl::ConditionAnd<PointT>);
    pcl::PackedRGBComparison<PointT>::ConstPtr min_r_cond = pcl::PackedRGBComparison<PointT>::ConstPtr (new pcl::PackedRGBComparison<PointT> ("r", pcl::ComparisonOps::GT, min_r));
    pcl::PackedRGBComparison<PointT>::ConstPtr max_r_cond = pcl::PackedRGBComparison<PointT>::ConstPtr (new pcl::PackedRGBComparison<PointT> ("r", pcl::ComparisonOps::LT, max_r));
    pcl::PackedRGBComparison<PointT>::ConstPtr min_g_cond = pcl::PackedRGBComparison<PointT>::ConstPtr (new pcl::PackedRGBComparison<PointT> ("g", pcl::ComparisonOps::GT, min_g));
    pcl::PackedRGBComparison<PointT>::ConstPtr max_g_cond = pcl::PackedRGBComparison<PointT>::ConstPtr (new pcl::PackedRGBComparison<PointT> ("g", pcl::ComparisonOps::LT, max_g));
    pcl::PackedRGBComparison<PointT>::ConstPtr min_b_cond = pcl::PackedRGBComparison<PointT>::ConstPtr (new pcl::PackedRGBComparison<PointT> ("b", pcl::ComparisonOps::GT, min_b));
    pcl::PackedRGBComparison<PointT>::ConstPtr max_b_cond = pcl::PackedRGBComparison<PointT>::ConstPtr (new pcl::PackedRGBComparison<PointT> ("b", pcl::ComparisonOps::LT, max_b));

    color_cond->addComparison (min_r_cond); 
    color_cond->addComparison (max_r_cond);
    color_cond->addComparison (min_g_cond);
    color_cond->addComparison (max_g_cond);
    color_cond->addComparison (min_b_cond); 
    color_cond->addComparison (max_b_cond);
    
    *cloud_inside_of_cup = this->filterColor(color_cond, cloud_cup);
    
    *cloud_inside_of_cup = this->removeNaNs(this->pass, cloud_inside_of_cup);
    
    double a = coefficients_plane->values[0];
    double b = coefficients_plane->values[1];
    double c = coefficients_plane->values[2];
    double d = coefficients_plane->values[3];
    std::cout << cloud_inside_of_cup->points.size() << "\n";
    if (cloud_inside_of_cup->points.size() != 0){
        for (auto& point: *cloud_inside_of_cup)
        {
            distances.push_back(std::abs(a*point.x + b*point.y + c*point.z + d) / std::sqrt(a*a + b*b + c*c));
        }
        double max = *std::max_element(distances.begin(), distances.end());
        std_msgs::Float64 max_distance;
        max_distance.data = max;
        this->height_pub.publish(max_distance);
        distances.clear();
    }
    
    
    pcl::toROSMsg(*cloud_inside_of_cup, test_cloud);
    test_cloud.header.frame_id = "zed_left_camera_frame";
    test_cloud.header.stamp = ros::Time::now();
    this->test_cloud_pub.publish(test_cloud);
    
    this->clearClouds();
}