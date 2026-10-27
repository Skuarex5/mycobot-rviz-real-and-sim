#pragma once
#include "mycobot_280_live_control/planning_marker_visibility.hpp"
#include <QElapsedTimer>
#include <QTimer>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
class QPushButton;
class QLabel;
class QSlider;
namespace mycobot_280_live_control {
class LiveFollowPanel : public rviz_common::Panel {
  Q_OBJECT
public:
  explicit LiveFollowPanel(QWidget *parent = nullptr);
  ~LiveFollowPanel() override;

private:
  void poll();
  void updatePlanningMarker();
  PlanningMarkerVisibility planning_marker_visibility_;
  QPushButton *button_;
  QPushButton *physical_button_;
  QLabel *destination_;
  bool physical_ = false, selecting_ = false, have_coordinator_ = false;
  QElapsedTimer selection_age_, coordinator_age_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr selection_client_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr physical_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr destination_sub_;
  std::optional<rclcpp::Client<std_srvs::srv::SetBool>::FutureAndRequestId>
      selection_future_;
  QLabel *status_;
  QLabel *grip_label_;
  QSlider *slider_;
  QTimer timer_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr heartbeat_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enabled_sub_,
      variant_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr grip_sub_;
  bool enabled_ = false, has_gripper_ = false, have_grip_ = false,
       pending_ = false, have_backend_ = false;
  QElapsedTimer backend_age_, request_age_;
  std::optional<rclcpp::Client<std_srvs::srv::SetBool>::FutureAndRequestId>
      future_;
};
} // namespace mycobot_280_live_control
