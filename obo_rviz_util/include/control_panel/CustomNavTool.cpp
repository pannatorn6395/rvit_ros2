#include "CustomNavTool.h"
#include <rviz_common/display_context.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace obo_rviz_util {

CustomNavTool::CustomNavTool() {
  // Constructor initialization if necessary
}

CustomNavTool::~CustomNavTool() {
  // Destructor cleanup if necessary
}

void CustomNavTool::onInitialize() {
  PoseTool::onInitialize();
  pub_ = nh_->create_publisher<geometry_msgs::msg::PoseStamped>("custom_nav_goal", 1);
}

void CustomNavTool::onPoseSet(double x, double y, double theta) {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = context_->getFixedFrame().toStdString();
  pose.header.stamp = nh_->now();
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), theta));

  pub_->publish(pose);
}

} // end namespace obo_rviz_util

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(obo_rviz_util::CustomNavTool, rviz_common::Tool)
