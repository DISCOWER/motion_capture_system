// Some functionality that is not defined in tf2 yet
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

void quaternionEigenToMsg(const Eigen::Quaterniond& eigen, geometry_msgs::msg::Quaternion& msg) {
  msg.x = eigen.x();
  msg.y = eigen.y();
  msg.z = eigen.z();
  msg.w = eigen.w();
}

void pointEigenToMsg(const Eigen::Vector3d& eigen, geometry_msgs::msg::Point& msg) {
  msg.x = eigen.x();
  msg.y = eigen.y();
  msg.z = eigen.z();
}

void vectorEigenToMsg(const Eigen::Vector3d& eigen, geometry_msgs::msg::Vector3& msg) {
  msg.x = eigen.x();
  msg.y = eigen.y();
  msg.z = eigen.z();
}