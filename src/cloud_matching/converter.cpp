#include "cloud_matching/converter.h"

Eigen::Isometry3d
toIsometry(Eigen::Matrix4f mat)
{
  Eigen::Matrix4d mat_double = mat.cast<double>();
  Eigen::Isometry3d iso;
  iso = mat_double;
  return iso;
}

Eigen::Matrix4f
toMatrix(Eigen::Isometry3d iso)
{
  Eigen::Matrix4d mat_double = iso.matrix();
  Eigen::Matrix4f mat = mat_double.cast<float>();
  return mat;
}

Eigen::Matrix4f
toMatrix(nav_msgs::Odometry msg)
{
  Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
  mat(0, 3) = msg.pose.pose.position.x;
  mat(1, 3) = msg.pose.pose.position.y;
  mat(2, 3) = msg.pose.pose.position.z;

  Eigen::Quaternionf q;
  q.x() = msg.pose.pose.orientation.x;
  q.y() = msg.pose.pose.orientation.y;
  q.z() = msg.pose.pose.orientation.z;
  q.w() = msg.pose.pose.orientation.w;

  Eigen::Matrix3f rot = q.normalized().toRotationMatrix();
  mat.block<3, 3>(0, 0) = rot;
  return mat;
}

Eigen::Matrix4f
toMatrix(Eigen::VectorXd v)
{
  Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
  mat(0, 3) = v(5);
  mat(1, 3) = v(6);
  mat(2, 3) = v(7);
  Eigen::Quaternionf q;
  q.x() = v(1);
  q.y() = v(2);
  q.z() = v(3);
  q.w() = v(0);
  Eigen::Matrix3f rot = q.normalized().toRotationMatrix();
  mat.block<3, 3>(0, 0) = rot;
  return mat;
}

Eigen::Vector3d
toVector(sensor_msgs::Imu imu)
{
  Eigen::Vector3d v;
  v(0) = imu.linear_acceleration.x;
  v(1) = imu.linear_acceleration.y;
  v(2) = imu.linear_acceleration.z;
  return v;
}

Eigen::Vector3f
toRPY(Eigen::Matrix4f mat)
{
  return mat.block<3, 3>(0, 0).eulerAngles(0, 1, 2);
}

geometry_msgs::Pose
toPose(Eigen::Matrix4f mat)
{
  geometry_msgs::Pose p;
  p.position.x = mat(0, 3);
  p.position.y = mat(1, 3);
  p.position.z = mat(2, 3);

  Eigen::Quaternionf q(mat.block<3, 3>(0, 0));
  p.orientation.x = q.x();
  p.orientation.y = q.y();
  p.orientation.z = q.z();
  p.orientation.w = q.w();
  return p;
}

geometry_msgs::Point
toPoint(Eigen::Matrix4f mat)
{
  geometry_msgs::Point p;
  p.x = mat(0, 3);
  p.y = mat(1, 3);
  p.z = mat(2, 3);
  return p;
}
