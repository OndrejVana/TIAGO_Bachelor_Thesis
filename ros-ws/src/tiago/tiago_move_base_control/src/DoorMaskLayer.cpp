#include <tiago_move_base_control/DoorMaskLayer.h>

#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include <cmath>
#include <algorithm>

PLUGINLIB_EXPORT_CLASS(tiago_move_base_control::DoorMaskLayer, costmap_2d::Layer)

namespace tiago_move_base_control
{

DoorMaskLayer::DoorMaskLayer()
: enabled_(false),
  width_m_(1.1),
  thickness_m_(0.35),
  margin_m_(0.10),
  set_unknown_(false),
  have_pose_(false),
  last_min_x_(0), last_min_y_(0), last_max_x_(0), last_max_y_(0)
{}

void DoorMaskLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);

  nh.param("door_pose_topic", door_pose_topic_, std::string("/door/door_pose_map"));
  nh.param("enabled_topic", enabled_topic_, std::string("/door_mask/enabled"));
  nh.param("enabled", enabled_, false);
  nh.param("width_m", width_m_, 1.1);
  nh.param("thickness_m", thickness_m_, 0.35);
  nh.param("margin_m", margin_m_, 0.10);
  nh.param("set_unknown_instead_of_free", set_unknown_, false);

  door_pose_sub_ = nh.subscribe(door_pose_topic_, 1, &DoorMaskLayer::doorPoseCb, this);
  enabled_sub_   = nh.subscribe(enabled_topic_,   1, &DoorMaskLayer::enabledCb, this);

  current_ = true;

  ROS_INFO_STREAM("DoorMaskLayer initialized. pose_topic=" << door_pose_topic_
                  << " enabled_topic=" << enabled_topic_
                  << " enabled=" << enabled_);
}

void DoorMaskLayer::enabledCb(const std_msgs::BoolConstPtr& msg)
{
  enabled_ = msg->data;
}

void DoorMaskLayer::doorPoseCb(const geometry_msgs::PoseStampedConstPtr& msg)
{
  last_pose_ = *msg;
  have_pose_ = true;

  const double x = msg->pose.position.x;
  const double y = msg->pose.position.y;
  const double yaw = tf2::getYaw(msg->pose.orientation);

  const double hw = 0.5 * width_m_ + margin_m_;
  const double ht = 0.5 * thickness_m_ + margin_m_;

  const double c = std::cos(yaw);
  const double s = std::sin(yaw);

  const double ex = std::abs(c) * hw + std::abs(s) * ht;
  const double ey = std::abs(s) * hw + std::abs(c) * ht;

  last_min_x_ = x - ex;
  last_max_x_ = x + ex;
  last_min_y_ = y - ey;
  last_max_y_ = y + ey;
}

void DoorMaskLayer::updateBounds(double, double, double,
                                 double* min_x, double* min_y, double* max_x, double* max_y)
{
  if (!enabled_ || !have_pose_) return;

  *min_x = std::min(*min_x, last_min_x_);
  *min_y = std::min(*min_y, last_min_y_);
  *max_x = std::max(*max_x, last_max_x_);
  *max_y = std::max(*max_y, last_max_y_);
}

void DoorMaskLayer::updateCosts(costmap_2d::Costmap2D& master_grid,
                                int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_ || !have_pose_) return;

  const double cx  = last_pose_.pose.position.x;
  const double cy  = last_pose_.pose.position.y;
  const double yaw = tf2::getYaw(last_pose_.pose.orientation);

  const double hw = 0.5 * width_m_ + margin_m_;
  const double ht = 0.5 * thickness_m_ + margin_m_;

  const double c = std::cos(yaw);
  const double s = std::sin(yaw);

  min_i = std::max(min_i, 0);
  min_j = std::max(min_j, 0);
  max_i = std::min(max_i, (int)master_grid.getSizeInCellsX());
  max_j = std::min(max_j, (int)master_grid.getSizeInCellsY());

  const unsigned char val = set_unknown_ ? costmap_2d::NO_INFORMATION : costmap_2d::FREE_SPACE;

  for (int j = min_j; j < max_j; ++j)
  {
    for (int i = min_i; i < max_i; ++i)
    {
      double wx, wy;
      master_grid.mapToWorld(i, j, wx, wy);

      const double dx = wx - cx;
      const double dy = wy - cy;

      // rotate by -yaw into door frame
      const double lx =  c * dx + s * dy;
      const double ly = -s * dx + c * dy;

      if (std::abs(lx) <= hw && std::abs(ly) <= ht)
      {
        master_grid.setCost(i, j, val);
      }
    }
  }
}

void DoorMaskLayer::reset()
{
  have_pose_ = false;
}

}  // namespace tiago_move_base_control