#ifndef MERGE_MAPS_3D_CONVERTER_H_
#define MERGE_MAPS_3D_CONVERTER_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

Eigen::Isometry3d toIsometry(Eigen::Matrix4f mat);
Eigen::Matrix4f toMatrix(Eigen::Isometry3d iso);
Eigen::Matrix4f toMatrix(nav_msgs::Odometry msg);
Eigen::Matrix4f toMatrix(Eigen::VectorXd v);

Eigen::Vector3d toVector(sensor_msgs::Imu imu);
Eigen::Vector3f toRPY(Eigen::Matrix4f mat);

geometry_msgs::Pose toPose(Eigen::Matrix4f mat);
geometry_msgs::Point toPoint(Eigen::Matrix4f mat);
#endif
