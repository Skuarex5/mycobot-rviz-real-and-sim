#include "mycobot_280_live_control/live_follow_panel.hpp"
#include "mycobot_280_live_control/planning_marker_visibility.hpp"
#include <QApplication>
#include <QPushButton>
#include <QSlider>
#include <QTest>
#include <cassert>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/properties/bool_property.hpp>
int main(int argc, char **argv) {
  QApplication app(argc, argv);
  rclcpp::init(argc, argv);
  {
    rviz_common::properties::Property request("Planning Request");
    auto goal = new rviz_common::properties::BoolProperty("Query Goal State",
                                                          true, "", &request);
    mycobot_280_live_control::PlanningMarkerVisibility visibility;
    visibility.update(&request, true);
    assert(!goal->getValue().toBool());
    visibility.update(&request, true);
    visibility.update(&request, false);
    assert(goal->getValue().toBool());
    goal->setValue(false);
    visibility.update(&request, true);
    visibility.update(&request, false);
    assert(!goal->getValue().toBool());
    goal->setValue(true);
    {
      mycobot_280_live_control::PlanningMarkerVisibility temporary;
      temporary.update(&request, true);
    }
    assert(goal->getValue().toBool());
    visibility.update(&request, true);
    delete goal;
    visibility.update(nullptr, false);
  }
  auto fake = std::make_shared<rclcpp::Node>("live_panel_test_backend");
  auto qos = rclcpp::QoS(1).reliable().transient_local();
  auto enabled =
      fake->create_publisher<std_msgs::msg::Bool>("/mycobot_live/enabled", qos);
  auto variant = fake->create_publisher<std_msgs::msg::Bool>(
      "/mycobot_live/has_gripper", qos);
  auto position = fake->create_publisher<std_msgs::msg::Float64>(
      "/mycobot_live/gripper_position", qos);
  auto physical = fake->create_publisher<std_msgs::msg::Bool>(
      "/mycobot_control/physical", qos);
  int selections = 0;
  auto selection = fake->create_service<std_srvs::srv::SetBool>(
      "/mycobot_control/select_physical",
      [&](std_srvs::srv::SetBool::Request::SharedPtr req,
          std_srvs::srv::SetBool::Response::SharedPtr res) {
        ++selections;
        res->success = false;
        res->message = "USB no disponible";
      });
  int beats = 0, targets = 0, requests = 0;
  double last = 0;
  auto beat = fake->create_subscription<std_msgs::msg::Empty>(
      "/mycobot_live/heartbeat", 1,
      [&](std_msgs::msg::Empty::ConstSharedPtr) { ++beats; });
  auto target = fake->create_subscription<std_msgs::msg::Float64>(
      "/mycobot_live/gripper_target", 1,
      [&](std_msgs::msg::Float64::ConstSharedPtr m) {
        ++targets;
        last = m->data;
      });
  auto service = fake->create_service<std_srvs::srv::SetBool>(
      "/mycobot_live/set_enabled",
      [&](std_srvs::srv::SetBool::Request::SharedPtr,
          std_srvs::srv::SetBool::Response::SharedPtr res) {
        ++requests;
        res->success = false;
        res->message = "Test rejection";
      });
  auto spin = [&](int ms) {
    for (int i = 0; i < ms / 10; ++i) {
      rclcpp::spin_some(fake);
      app.processEvents();
      QTest::qWait(10);
    }
  };
  auto panel = std::make_unique<mycobot_280_live_control::LiveFollowPanel>();
  auto button = panel->findChild<QPushButton *>("toggle_follow");
  auto selector = panel->findChild<QPushButton *>("toggle_physical");
  assert(selector && selector->isCheckable());
  assert(!selector->isEnabled());
  auto slider = panel->findChild<QSlider *>("gripper_target");
  assert(button && button->text() == "Activar seguimiento");
  assert(slider && slider->isHidden());
  assert(!button->isEnabled());
  std_msgs::msg::Bool b;
  b.data = false;
  enabled->publish(b);
  physical->publish(b);
  variant->publish(b);
  std_msgs::msg::Float64 p;
  p.data = -.1;
  position->publish(p);
  spin(600);
  assert(button->isEnabled());
  assert(selector->isEnabled() && !selector->isChecked());
  selector->click();
  assert(!selector->isEnabled() && !selector->isChecked());
  spin(300);
  assert(selections == 1 && !selector->isChecked());
  std_msgs::msg::Bool selected;
  selected.data = true;
  physical->publish(selected);
  spin(200);
  assert(selector->isChecked());
  assert(targets == 0);
  assert(beats > 0);
  button->click();
  assert(!button->isEnabled());
  spin(300);
  assert(requests == 1);
  assert(button->text() == "Activar seguimiento");
  b.data = true;
  variant->publish(b);
  enabled->publish(b);
  spin(300);
  assert(!slider->isHidden());
  assert(slider->isEnabled());
  assert(!selector->isEnabled());
  assert(targets == 0);
  slider->setValue(600);
  spin(200);
  assert(targets == 1);
  assert(std::abs(last - .1) < 1e-10);
  slider->setValue(650);
  spin(200);
  assert(targets == 2);
  assert(last == .15);
  button->click();
  assert(!button->isEnabled());
  panel.reset();
  spin(200);
  int stopped = beats;
  spin(300);
  assert(beats == stopped);
  rclcpp::shutdown();
}
