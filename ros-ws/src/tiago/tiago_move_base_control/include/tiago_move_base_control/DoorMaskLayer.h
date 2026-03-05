#pragma once

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <string>

namespace tiago_move_base_control
{

class DoorMaskLayer : public costmap_2d::Layer
{
public:
  DoorMaskLayer();

  void onInitialize() override;

  void updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double* min_x, double* min_y, double* max_x, double* max_y) override;

  void updateCosts(costmap_2d::Costmap2D& master_grid,
                   int min_i, int min_j, int max_i, int max_j) override;

  void reset() override;

private:
  void doorPoseCb(const geometry_msgs::PoseStampedConstPtr& msg);
  void enabledCb(const std_msgs::BoolConstPtr& msg);

  // params
  std::string door_pose_topic_;
  std::string enabled_topic_;
  bool enabled_;
  double width_m_;
  double thickness_m_;
  double margin_m_;
  bool set_unknown_;

  // state
  ros::Subscriber door_pose_sub_;
  ros::Subscriber enabled_sub_;
  geometry_msgs::PoseStamped last_pose_;
  bool have_pose_;

  // cached AABB for updateBounds
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
};

}  // namespace tiago_move_base_control