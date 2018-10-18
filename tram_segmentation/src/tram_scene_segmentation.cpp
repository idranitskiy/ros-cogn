#include "ros/ros.h"
#include "std_msgs/String.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PolygonStamped.h>


#include <vector>


#include "tsvio.h"


#include "helpers.cpp"





bool visualizerInitialized(false);

boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  static boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));  



  if (!visualizerInitialized)
  {
    viewer->setBackgroundColor (0, 0, 0);
//    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZI> rgb(cloud);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> point_cloud_color_handler(cloud,"intensity");
    viewer->addPointCloud<pcl::PointXYZI> (cloud,point_cloud_color_handler, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addPointCloudNormals<pcl::PointXYZI, pcl::Normal> (cloud, normals, 10, 0.5, "normals");
    viewer->addCoordinateSystem (1.0);
    viewer->setCameraPosition(0,0,50,
                              0,0,1,
                              1,0,0);
    visualizerInitialized = true;
  }
  else
  {
    //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZI> rgb(cloud);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> point_cloud_color_handler(cloud,"intensity");
    viewer->updatePointCloud(cloud,point_cloud_color_handler, "sample cloud");

    viewer->removePointCloud("normals", 0);
    viewer->addPointCloudNormals<pcl::PointXYZI, pcl::Normal> (cloud, normals, 1, 0.5, "normals");
  }

  viewer->spinOnce(100);
  return (viewer);
}





//Global Publishers/Subscribers
ros::Subscriber subPointCloud;
ros::Publisher voxelGridCloudPublisher;
ros::Publisher passThroughFilterPublisher_height;
ros::Publisher passThroughFilterPublisher_cabin;
ros::Publisher groundCloudPublisher;
ros::Publisher nonGroundPublisher;
ros::Publisher clustersPublisher;
ros::Publisher projectedClustersPublisher;

ros::Publisher markerArrayPublisher;



//Global intermediate results
sensor_msgs::PointCloud2 voxelGridMsg;
sensor_msgs::PointCloud2 passThroughFilterMsg_height;
sensor_msgs::PointCloud2 passThroughFilterMsg_cabin;
sensor_msgs::PointCloud2 groundCloudMsg;
sensor_msgs::PointCloud2 nonGroundCloudMsg;
sensor_msgs::PointCloud2 clusters;
sensor_msgs::PointCloud2 projectedClusters;


ar::TsvWriter planeWriter;
ar::TsvWriter::Header planeHeader;

ar::TsvWriter bboxWriter;
ar::TsvWriter::Header bboxHeader;


void TramSceneSegmentation(const sensor_msgs::PointCloud2::ConstPtr& pointCloudMsg)
//void test_function(const std_msgs::String::ConstPtr& msg)
{
  ROS_DEBUG("Point Cloud Function Received");
  ROS_DEBUG("Velodyne_Points time stamp = %d", (int)pointCloudMsg->header.stamp.sec);
  ROS_DEBUG("Velodyne_Points time stamp in nano seconds = %d", (int)pointCloudMsg->header.stamp.nsec);
  ROS_DEBUG("Velodyne Points sequence ID = %d", (int)pointCloudMsg->header.seq);

  
  visualization_msgs::MarkerArray markerArrayMsg;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);

  pcl::fromROSMsg(*pointCloudMsg, *cloud);



  //Do voxel grid downsampling
  pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_grid (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::VoxelGrid<pcl::PointXYZI> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(0.2f, 0.2f, 0.2f);
  vg.filter(*voxel_grid);

  pcl::toROSMsg(*(voxel_grid.get()), voxelGridMsg);
  voxelGridCloudPublisher.publish(voxelGridMsg);
  /// Result of voxel grid downsampling published

  //Find normals for downsampled cloud
  pcl::PointCloud<pcl::Normal>::Ptr voxel_grid_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normal_estimation;
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>());

  normal_estimation.setInputCloud(voxel_grid);
  normal_estimation.setSearchMethod(tree);
  normal_estimation.setRadiusSearch(0.5);
  normal_estimation.compute(*voxel_grid_normals);

  //Remove points inside cabin and points that are very high
  pcl::PointCloud<pcl::PointXYZI>::Ptr pass_through_filter_height (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr pass_through_filter_cabin (new pcl::PointCloud<pcl::PointXYZI>);
  
  //Prepare heigh condition
  pcl::ConditionAnd<pcl::PointXYZI>::Ptr height_cond (new pcl::ConditionAnd<pcl::PointXYZI>());
  height_cond->addComparison (pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZI>("z", pcl::ComparisonOps::GT, -2.0)));
  height_cond->addComparison (pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZI>("z", pcl::ComparisonOps::LT, 4.0)));

  //Prepare cabin condition
  pcl::ConditionOr<pcl::PointXYZI>::Ptr cabin_cond (new pcl::ConditionOr<pcl::PointXYZI>());
  cabin_cond->addComparison (pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::GT, 2.0)));
  cabin_cond->addComparison (pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::LT, -2.0)));
  cabin_cond->addComparison (pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::GT, 2.0)));
  cabin_cond->addComparison (pcl::FieldComparison<pcl::PointXYZI>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::LT, -5.0)));

  //Remove points that a very high
  pcl::ConditionalRemoval<pcl::PointXYZI> height_condrem;
  height_condrem.setCondition(height_cond);
  height_condrem.setInputCloud(voxel_grid);
  height_condrem.setKeepOrganized(true);
  height_condrem.filter(*pass_through_filter_height);

  pcl::toROSMsg(*(pass_through_filter_height.get()),passThroughFilterMsg_height);
  passThroughFilterPublisher_height.publish(passThroughFilterMsg_height);

  //Remove points inside the cabin
  pcl::ConditionalRemoval<pcl::PointXYZI> cabin_condrem;
  cabin_condrem.setCondition(cabin_cond);
  cabin_condrem.setInputCloud(pass_through_filter_height);
  cabin_condrem.setKeepOrganized(true);
  cabin_condrem.filter(*pass_through_filter_cabin);

  pcl::toROSMsg(*(pass_through_filter_cabin.get()), passThroughFilterMsg_cabin);
  passThroughFilterPublisher_cabin.publish(passThroughFilterMsg_cabin);
  //Pass through data published

  //plane segmentation
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr non_ground_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointIndices::Ptr plane_inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr plane_model_coefficients (new pcl::ModelCoefficients);

  pcl::SACSegmentation<pcl::PointXYZI> plane_seg;
  plane_seg.setInputCloud(pass_through_filter_cabin);
  //plane_seg.setInputNormals(voxel_grid_normals);
  plane_seg.setModelType(pcl::SACMODEL_PLANE);
  plane_seg.setMethodType(pcl::SAC_RANSAC);
  //plane_seg.setNormalDistanceWeight(0.05);
  //plane_seg.setDistanceFromOrigin(1.6);
  plane_seg.setDistanceThreshold(0.4);
  plane_seg.setMaxIterations(1000);
  plane_seg.setOptimizeCoefficients(true);
  plane_seg.setAxis(Eigen::Vector3f(0,0,1));
  plane_seg.segment(*plane_inliers, *plane_model_coefficients);

  
  /*
  //TODO dump plane model coeffs here
  std::vector<ar::Value> planeRow;
  planeRow.emplace_back(static_cast<int64_t>(pointCloudMsg->header.stamp.sec));
  planeRow.emplace_back(static_cast<int64_t>(pointCloudMsg->header.stamp.nsec));
  planeRow.emplace_back(static_cast<int64_t>(pointCloudMsg->header.seq));
  planeRow.emplace_back(plane_model_coefficients->values[0]);
  planeRow.emplace_back(plane_model_coefficients->values[1]);
  planeRow.emplace_back(plane_model_coefficients->values[2]);
  planeRow.emplace_back(plane_model_coefficients->values[3]);
  planeWriter.writeRow(planeRow);
  */

  //Extract points corresponding to ground and non ground
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(pass_through_filter_cabin);
  extract.setIndices(plane_inliers);
  extract.setNegative(false);
  extract.filter(*ground_cloud);
  extract.setNegative(true);
  extract.filter(*non_ground_cloud);

  
  
  

  pcl::toROSMsg(*(ground_cloud.get()), groundCloudMsg);
  groundCloudPublisher.publish(groundCloudMsg);

  pcl::toROSMsg(*(non_ground_cloud.get()), nonGroundCloudMsg);
  nonGroundPublisher.publish(nonGroundCloudMsg);

  //Remove NANs from point cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr non_ground_cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
  std::vector<int> indexes;
  pcl::removeNaNFromPointCloud(*non_ground_cloud, *non_ground_cloud_filtered, indexes);

  

  //Do Euclidean Cluster Extraction
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree_non_flat (new pcl::search::KdTree<pcl::PointXYZI>);
  tree_non_flat->setInputCloud(non_ground_cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance(0.8);
  ec.setMinClusterSize(15);
  ec.setMaxClusterSize(1000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(non_ground_cloud_filtered);
  ec.extract(cluster_indices);
  ROS_DEBUG("Number of extracted clusters = %d", (int)cluster_indices.size());

  visualization_msgs::Marker marker;

  marker.action = visualization_msgs::Marker::DELETEALL;
  markerArrayMsg.markers.push_back(marker);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr clustered_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected_clustered_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
      int r = std::rand() % 254;
      int g = std::rand() % 254;
      int b = std::rand() % 254;
      int32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
    {
      pcl::PointXYZRGB tempPoint;
      tempPoint.x = non_ground_cloud_filtered->points[*pit].x;
      tempPoint.y = non_ground_cloud_filtered->points[*pit].y;
      tempPoint.z = non_ground_cloud_filtered->points[*pit].z;
      tempPoint.rgb = rgb;

      cloud_cluster->points.push_back(tempPoint);
      cloud_cluster->width = cloud_cluster->points.size();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = false;
    }

    pcl::ProjectInliers<pcl::PointXYZRGB> proj;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(cloud_cluster);
    proj.setModelCoefficients(plane_model_coefficients);
    proj.filter(*cloud_projected);

    *projected_clustered_cloud += *cloud_projected;

    *cloud_cluster += *cloud_projected;


    Eigen::Vector3f markerTransform(0,0,0);
    Eigen::Quaternionf markerQuaternion(0,0,0,1);
    Eigen::Vector3f markerScale(0,0,0);
    Eigen::Matrix3f markerMoments;
    markerMoments << 1,0,0, 0,1,0, 0,0,1;

    get3DBoundingBox(cloud_cluster, markerTransform, markerQuaternion, markerScale, markerMoments);

    marker.header = pointCloudMsg->header;
    marker.ns = "pointcloud clusters";
    marker.id = (*it).indices[0];

    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = markerTransform[0];
    marker.pose.position.y = markerTransform[1];
    marker.pose.position.z = markerTransform[2];

    marker.pose.orientation.x = markerQuaternion.x();
    marker.pose.orientation.y = markerQuaternion.y();
    marker.pose.orientation.z = markerQuaternion.z();
    marker.pose.orientation.w = markerQuaternion.w();

    marker.scale.x = markerScale[0];
    marker.scale.y = markerScale[1];
    marker.scale.z = markerScale[2];


    marker.color.r = r/255.0;
    marker.color.g = g/255.0;
    marker.color.b = b/255.0;
    marker.color.a = 1.0;
/*
    std::vector<ar::Value> bboxRow;
    bboxRow.emplace_back(static_cast<int64_t>(pointCloudMsg->header.stamp.sec));
    bboxRow.emplace_back(static_cast<int64_t>(pointCloudMsg->header.stamp.nsec));
    bboxRow.emplace_back(static_cast<int64_t>(pointCloudMsg->header.seq));
    bboxRow.emplace_back(-marker.pose.position.y );
    bboxRow.emplace_back(marker.pose.position.x );
    bboxRow.emplace_back(marker.pose.position.z );
 
    bboxRow.emplace_back(markerQuaternion.matrix().eulerAngles(2,1,0)[0]); //yaw
    bboxRow.emplace_back(markerQuaternion.matrix().eulerAngles(2,1,0)[1]); //pitch
    bboxRow.emplace_back(markerQuaternion.matrix().eulerAngles(2,1,0)[2]); //roll
 
    bboxRow.emplace_back(marker.scale.x);
    bboxRow.emplace_back(marker.scale.y);
    bboxRow.emplace_back(marker.scale.z);
 
    bboxRow.emplace_back(marker.color.r);
    bboxRow.emplace_back(marker.color.g);
    bboxRow.emplace_back(marker.color.b);
    bboxRow.emplace_back(marker.color.a);
 
    bboxRow.emplace_back(marker.id);
    bboxWriter.writeRow(bboxRow);
    */

    markerArrayMsg.markers.push_back(marker);

    ROS_DEBUG("Number of points in cluster = %d", (int)cloud_cluster->points.size());
    *clustered_cloud += *cloud_cluster;
  }

  pcl::toROSMsg(*(projected_clustered_cloud.get()), projectedClusters);
  projectedClusters.header.stamp = ros::Time::now();
  projectedClusters.header.frame_id = "velodyne";
  projectedClustersPublisher.publish(projectedClusters);
  
  pcl::toROSMsg(*(clustered_cloud.get()), clusters);
  clusters.header.stamp = ros::Time::now();
  clusters.header.frame_id = "velodyne";

  clustersPublisher.publish(clusters);

  markerArrayPublisher.publish(markerArrayMsg);
}


int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");

/*
  std::cout << "Arguments:" << std::endl;
  for(auto i = 0; i < argc; i++)
  {
    std::cout << argv[i] << std::endl;
  }
  
  std::string season = argv[1];
  std::string episode = argv[2];

  planeHeader.emplace_back("second", ar::Value::INT64);
  planeHeader.emplace_back("nanosecond", ar::Value::INT64);
  planeHeader.emplace_back("ros_frame_number", ar::Value::INT64);

  planeHeader.emplace_back("a", ar::Value::FLOAT64);
  planeHeader.emplace_back("b", ar::Value::FLOAT64);
  planeHeader.emplace_back("c", ar::Value::FLOAT64);
  planeHeader.emplace_back("d", ar::Value::FLOAT64);
  planeWriter.setHeader(planeHeader);
  //testdata/srl/out.ldr/srl.szn/srl.szn.epi.ldr.gnd.tsv
  planeWriter.open("/media/sf_testdata/trm/out.ldr/trm.052/" + season + "." + episode + ".ldr.gnd.tsv");


  bboxHeader.emplace_back("second", ar::Value::INT64);
  bboxHeader.emplace_back("nanosecond", ar::Value::INT64);
  bboxHeader.emplace_back("ros_frame_number", ar::Value::INT64);

  bboxHeader.emplace_back("transition.x", ar::Value::FLOAT64);
  bboxHeader.emplace_back("transition.y", ar::Value::FLOAT64);
  bboxHeader.emplace_back("transition.z", ar::Value::FLOAT64);

  bboxHeader.emplace_back("orientation.yaw", ar::Value::FLOAT64);
  bboxHeader.emplace_back("orientation.pitch", ar::Value::FLOAT64);
  bboxHeader.emplace_back("orientation.roll", ar::Value::FLOAT64);
  

  bboxHeader.emplace_back("scale.x", ar::Value::FLOAT64);
  bboxHeader.emplace_back("scale.y", ar::Value::FLOAT64);
  bboxHeader.emplace_back("scale.z", ar::Value::FLOAT64);

  bboxHeader.emplace_back("color.r", ar::Value::FLOAT64);
  bboxHeader.emplace_back("color.g", ar::Value::FLOAT64);
  bboxHeader.emplace_back("color.b", ar::Value::FLOAT64);
  bboxHeader.emplace_back("color.a", ar::Value::FLOAT64);

  bboxHeader.emplace_back("id", ar::Value::INT64);

  bboxWriter.setHeader(bboxHeader);
  ///out.ldr/srl.szn/srl.szn.epi.ldr.clu.tsv
  bboxWriter.open("/media/sf_testdata/trm/out.ldr/trm.052/" + season + "." + episode + ".ldr.clu.tsv");
*/

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
   subPointCloud = n.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 2, TramSceneSegmentation);
   voxelGridCloudPublisher = n.advertise<sensor_msgs::PointCloud2>("/voxel_grid/output", 1);
   passThroughFilterPublisher_height = n.advertise<sensor_msgs::PointCloud2>("/pass_through_filter_height/output",1);
   passThroughFilterPublisher_cabin = n.advertise<sensor_msgs::PointCloud2>("/pass_through_filter_cabin/output",1);
   groundCloudPublisher = n.advertise<sensor_msgs::PointCloud2>("/ground_cloud/output",1);
   nonGroundPublisher = n.advertise<sensor_msgs::PointCloud2>("/non_ground_cloud/output",1);
   clustersPublisher = n.advertise<sensor_msgs::PointCloud2>("/clusters/output", 1);
   projectedClustersPublisher = n.advertise<sensor_msgs::PointCloud2>("/projected_clusters",1);

   markerArrayPublisher = n.advertise<visualization_msgs::MarkerArray>("clusterMarker",1);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();
  

  return 0;
}
