**RUN**
Run velodyne_pointcloud launch file first:
roslaunch velodyne_pointcloud VLP16_points.launch pcap:=/home/ubuntu/bagfiles/trm.052. play_season:=true start:=5 end:=12


After, start tram_segmentation node:

rosrun tram_segmentation tram_scene_segmentation


Overview
========

Simple Euclidean segmentation based on PCL (Point Cloud Library).
Chunks /velodyne_points/ topic into point cloud cluster array for further tracking 

The following topics producted:
 - /voxel_grid/output - downsample of original input
 - /pass_through_filter_cabin/output - pass through filter to remove tram cabin interior
 - /pass_through_filter_height/output - pass through filter to remove points above 5 meters
 - /ground_cloud/output - points corresponding to ground plane after RANSAC plane segmentation
 - /non_ground_cloud/output - points above the ground after RANSAC plane segmentation
 - /clusters/output - Euclidean point clusters
 - /projected_clusters - clusters projected on the ground plane
 - /clusterMarker - 3D bounding boxes standing on the ground plane
