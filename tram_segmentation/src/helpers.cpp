


void get3DBoundingBox(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster, Eigen::Vector3f& transform, Eigen::Quaternionf& quaternion, Eigen::Vector3f& size, Eigen::Matrix3f& covariance)
{
  // Compute principal directions
  Eigen::Vector4f pcaCentroid;
  pcl::compute3DCentroid(*cluster, pcaCentroid);
  computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covariance);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
  Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
  eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

  Eigen::Vector3f angles = eigenVectorsPCA.eulerAngles(2, 1, 0);

    angles[1] = 0;
    angles[2] = 0;
  
  

  Eigen::AngleAxisf yawAngle(angles[0], Eigen::Vector3f::UnitZ());
  Eigen::AngleAxisf pitchAngle(angles[1], Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rollAngle(angles[2], Eigen::Vector3f::UnitX());

  Eigen::Quaternion<float> q = rollAngle*pitchAngle*yawAngle;

  eigenVectorsPCA = q.matrix();


  /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
  /// the signs are different and the box doesn't get correctly oriented in some cases.
  
  // Transform the original cloud to the origin where the principal components correspond to the axes.
  Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
  projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
  projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);
  // Get the minimum and maximum points of the transformed cloud.
  pcl::PointXYZRGB minPoint, maxPoint;
  pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
  const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());
  
  
  // Final transform
  quaternion = eigenVectorsPCA; //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
  transform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();
  size << maxPoint.x - minPoint.x, maxPoint.y - minPoint.y, maxPoint.z - minPoint.z;
}
