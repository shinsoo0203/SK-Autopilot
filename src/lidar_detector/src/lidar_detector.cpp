#include "lidar_detector.hpp"

using namespace std;

lidar_detector::lidar_detector()
{
    nh.param ("lidar_detector/leaf_size", m_fLeafSize, 0.1);
    nh.param ("lidar_detector/radius_search", m_fRadius, 0.8);
    nh.param ("lidar_detector/Marker_duration", m_fMarkerDuration, 0.1);
    nh.param ("lidar_detector/Voxel_leafsize", m_fLeafSize, 0.1);
    nh.param ("lidar_detector/threshold_range", m_dRange_m, 15.0);
    nh.param ("lidar_detector/cluster_err_range", m_dClusterErrRadius, 0.5);
    nh.param ("lidar_detector/cluster_min_size", m_dClusterMinSize, 5.0);
    nh.param ("lidar_detector/cluster_max_size", m_dClusterMaxSize, 20.0);
    nh.param("lidar_detector/remove_side_range", m_dRemoveSideRange, 3.0);


    pub_shape = nh.advertise<visualization_msgs::MarkerArray>("Shape", 1);
    pub_origin = nh.advertise<visualization_msgs::Marker> ("Origin", 1);
    pub_object = nh.advertise<sk_msgs::ObjectArray> ("objects",1);
    pub_object_marker = nh.advertise<visualization_msgs::MarkerArray>("object_marker",1);

    ydlidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, &lidar_detector::ydlidar_callback, this);

    pcl_from_scan = nh.advertise<PointCloud>("ydlidar_points", 1);
    filter_points = nh.advertise<PointCloud>("filtered_points", 1);
}

lidar_detector::~lidar_detector(){}

void lidar_detector::mainLoop()
{
    ros::Rate loop_rate(50);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void lidar_detector::ydlidar_callback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    sensor_msgs::PointCloud2 cloud;
    projector.projectLaser(*scan_in, cloud);

    // Publish the new point cloud.
    cloud.header.frame_id = "/fcu";
    cloud.header.stamp = scan_in->header.stamp;
    pcl_from_scan.publish(cloud);

    PointCloud::Ptr pInputCloud (new PointCloud);
    pcl::fromROSMsg(cloud, *pInputCloud);

    PointCloud::Ptr pDownsampledCloud (new PointCloud);
    downsample(pInputCloud, pDownsampledCloud, m_fLeafSize, m_fRadius);
    filter_points.publish(pDownsampledCloud);

    vector<pcl::PointIndices> vecClusterIndices;
    dbscan(pDownsampledCloud, vecClusterIndices);

    setCluster (vecClusterIndices, m_OriginalClusters, pDownsampledCloud);

    // Display shape
    displayShape (m_OriginalClusters);

    // publish
    publish();
}

void lidar_detector::downsample(const PointCloud::ConstPtr& pInputCloud, PointCloud::Ptr& pDownsampledCloud, float f_paramLeafSize_m, float f_radius_size)
{
    pDownsampledCloud->clear();

    // Voxel length of the corner : fLeafSize
    pcl::VoxelGrid<pcl::PointXYZ> voxelFilter;
    voxelFilter.setInputCloud (pInputCloud);
    voxelFilter.setLeafSize(f_paramLeafSize_m, f_paramLeafSize_m, f_paramLeafSize_m);
    voxelFilter.filter (*pDownsampledCloud);

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    // build the filter
    outrem.setInputCloud(pDownsampledCloud);
    outrem.setRadiusSearch(f_radius_size);
    outrem.setMinNeighborsInRadius (2);
    // apply filter
    outrem.filter (*pDownsampledCloud);
}

void lidar_detector::dbscan(const PointCloud::ConstPtr& pInputCloud, std::vector<pcl::PointIndices>& vecClusterIndices)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr pKdtreeDownsampledCloud (new pcl::search::KdTree<pcl::PointXYZ>);
    pKdtreeDownsampledCloud->setInputCloud (pInputCloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclideanCluster;
    euclideanCluster.setClusterTolerance (m_dClusterErrRadius);
    euclideanCluster.setMinClusterSize (m_dClusterMinSize);
    euclideanCluster.setMaxClusterSize (m_dClusterMaxSize);
    euclideanCluster.setSearchMethod (pKdtreeDownsampledCloud);
    euclideanCluster.setInputCloud (pInputCloud);
    euclideanCluster.extract (vecClusterIndices);
}

void lidar_detector::setCluster (const std::vector<pcl::PointIndices> vecClusterIndices, std::vector<clusterPtr>& pOriginalClusters, const PointCloud::Ptr pInputCloud)
{
    pOriginalClusters.clear();

    int objectNumber = 0;
    for (const auto& clusterIndice : vecClusterIndices)
    {
        std::string label_ = "Obstacle ";
        std::string label = label_;
        label.append (std::to_string(objectNumber));

        // generate random color to globalRGB variable
        generateColor(vecClusterIndices.size());

        // pCluster is a local cluster
        clusterPtr pCluster (new Cluster());

        // Cloring and calculate the cluster center point and quaternion
        pCluster->SetCloud(pInputCloud, clusterIndice.indices, m_velodyne_header, objectNumber, m_globalRGB[objectNumber].m_r, m_globalRGB[objectNumber].m_g, m_globalRGB[objectNumber].m_b, label, true);

        pOriginalClusters.push_back(pCluster);
        objectNumber++;
    }
}

void lidar_detector::generateColor(size_t indexNumber)
{
    if (indexNumber > m_maxIndexNumber)
    {
        int addRGB = indexNumber - m_maxIndexNumber;
        m_maxIndexNumber = indexNumber;

        for (size_t i = 0; i < addRGB; i++)
        {
            uint8_t r = 1024 * rand () % 255;
            uint8_t g = 1024 * rand () % 255;
            uint8_t b = 1024 * rand () % 255;
            m_globalRGB.push_back(RGB(r, g, b));
        }
    }
}


void lidar_detector::displayShape (const std::vector<clusterPtr> pVecClusters)
{
    // origin
    m_Origin.header.frame_id = "/fcu";
    m_Origin.header.stamp = ros::Time::now();

    m_Origin.ns = "/origin";
    m_Origin.id = 0;

    m_Origin.type = visualization_msgs::Marker::SPHERE;

    m_Origin.action = visualization_msgs::Marker::ADD;

    m_Origin.pose.position.x = 0;
    m_Origin.pose.position.y = 0;
    m_Origin.pose.position.z = 0;
    m_Origin.pose.orientation.x = 0.0;
    m_Origin.pose.orientation.y = 0.0;
    m_Origin.pose.orientation.z = 0.0;
    m_Origin.pose.orientation.w = 1.0;

    m_Origin.scale.x = 0.5;
    m_Origin.scale.y = 0.5;
    m_Origin.scale.z = 0.5;

    m_Origin.color.r = 0.0f;
    m_Origin.color.g = 1.0f;
    m_Origin.color.b = 0.0f;
    m_Origin.color.a = 1.0;

    m_Origin.lifetime = ros::Duration();

//    // broadcast tf
//    static tf::TransformBroadcaster br;
//    tf::Transform transform;

//    transform.setOrigin( tf::Vector3(m_Origin.pose.position.x, m_Origin.pose.position.y, m_Origin.pose.position.z) );
//    tf::Quaternion q;
//    q.setRPY(0.0, 0.0, 0.0);
//    transform.setRotation(q);

//    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/laser_frame", "/base_footprint"));


    // tracking objects
    m_arrShapes.markers.clear();
    m_arrObjects.objects.clear();

    m_arrObjects.frame_id = "/fcu";

    uint32_t objectNumber = 0;
    for (auto pCluster : pVecClusters)
    {
        visualization_msgs::Marker shape;
        sk_msgs::Object object;

        shape.lifetime = ros::Duration(m_fMarkerDuration);
        shape.header.frame_id = "/fcu";
        //shape.color.b = (float)(*cluster_it)->m_b/255.0f;
        shape.color.r = 0.0;
        shape.color.g = 1.0;
        shape.color.b = 0.0;
        shape.color.a = 0.5;
        shape.type = visualization_msgs::Marker::LINE_STRIP;
        shape.action = visualization_msgs::Marker::ADD;
        shape.ns = "/Polygon";
        shape.scale.x = 0.08;

        bool isClusterValid = pCluster->m_valid_cluster;

        if (isClusterValid)
        {
            shape.id = objectNumber;
            object.id = objectNumber;
            object.frame_id = "/fcu";

            for (auto const &point: pCluster->m_polygon.polygon.points)
            {
                geometry_msgs:: Point tmpPoint;
                tmpPoint.x = point.x;
                tmpPoint.y = point.y;
                //                                tmpPoint.z = point.z;
                shape.points.push_back (tmpPoint);
            }

            m_arrShapes.markers.push_back(shape);

            // original
            // object.pose.position = pCluster->m_center.position;

            // tmp
            object.pose.position.x = pCluster->m_center.position.y;
            object.pose.position.y = -pCluster->m_center.position.x;
            object.pose.position.x -= 0.02*k;
            object.pose.position.y -= 0.02*k;
            m_arrObjects.objects.push_back(object);

//            shape.scale.x = 0.0;
//            shape.scale.y = 0.0;
//            shape.scale.z = 0.5;
//            shape.points.clear();
//            shape.pose.position = pCluster->m_center.position;
//            shape.pose.orientation = pCluster->m_center.orientation;
//            shape.color.r = shape.color.g = shape.color.b = 1.0;
//            shape.color.a = 1.0;
//            shape.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
//            shape.ns = "/Text";
//            //			double distance = sqrt(pow(shape.pose.position.x, 2.0) + pow (shape.pose.position.y, 2.0));
//            //			shape.text = std::to_string(distance);
//            shape.text = std::to_string(pCluster->m_id);
//            m_arrShapes.markers.push_back (shape);
        }
        objectNumber++;
    }
    k++;

    visualization_msgs::Marker object_marker;

    object_marker.header.frame_id = "/fcu";
    object_marker.header.stamp = ros::Time::now();

    object_marker.id = 0;
    object_marker.type = visualization_msgs::Marker::SPHERE;
    object_marker.action = visualization_msgs::Marker::ADD;
    object_marker.pose.position.x = 0;
    object_marker.pose.position.y = 0;
    object_marker.pose.position.z = 0;
    object_marker.pose.orientation.x = 0.0;
    object_marker.pose.orientation.y = 0.0;
    object_marker.pose.orientation.z = 0.0;
    object_marker.pose.orientation.w = 1.0;

    object_marker.scale.x = 0.5;
    object_marker.scale.y = 0.5;
    object_marker.scale.z = 0.5;

    object_marker.color.r = 0.0f;
    object_marker.color.g = 1.0f;
    object_marker.color.b = 1.0f;
    object_marker.color.a = 1.0;
    object_marker.lifetime = ros::Duration();

    m_arrObjects_marker.markers.clear();
    for (auto object : m_arrObjects.objects) {
        object_marker.pose = object.pose;
        m_arrObjects_marker.markers.push_back(object_marker);
        object_marker.id += 1;
    }

}


void lidar_detector::publish ()
{
    pub_shape.publish (m_arrShapes);
    pub_origin.publish(m_Origin);
    pub_object.publish(m_arrObjects);
    pub_object_marker.publish(m_arrObjects_marker);
}

