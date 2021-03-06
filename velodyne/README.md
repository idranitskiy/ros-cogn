**Run launch-file on multiple pcap episodes:**
roslaunch velodyne_pointcloud VLP16_points.launch pcap:=/home/ubuntu/bagfiles/trm.052. play_season:=true start:=5 end:=12



**To use PCAP timestamps**, please use use_pcap_time flag:
roslaunch velodyne_pointcloud VLP16_points.launch pcap:=/home/ubuntu/bagfiles/trm.052. play_season:=true start:=5 end:=12 use_pcap_time:="True"



Overview
========

Velodyne<sup>1</sup> is a collection of ROS<sup>2</sup> packages supporting `Velodyne high
definition 3D LIDARs`<sup>3</sup>.

**Warning**:

  The master branch normally contains code being tested for the next
  ROS release.  It will not always work with every previous release.
  To check out the source for the most recent release, check out the
  tag `velodyne-<version>` with the highest version number.

The current ``master`` branch works with ROS Indigo and Kinetic.
CI builds are currently run for Lunar and Melodic but extensive
testing has not been completed in those environments.

- <sup>1</sup>ROS: http://www.ros.org
- <sup>2</sup>Velodyne: http://www.ros.org/wiki/velodyne
- <sup>3</sup>`Velodyne high definition 3D LIDARs`: http://www.velodynelidar.com/lidar/lidar.aspx
