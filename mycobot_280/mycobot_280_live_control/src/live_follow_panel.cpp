#include "mycobot_280_live_control/live_follow_panel.hpp"
#include <rviz_common/tool.hpp>
#include <rviz_common/tool_manager.hpp>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSlider>
#include <QVBoxLayout>
#include <algorithm>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/display_group.hpp>
namespace mycobot_280_live_control {
LiveFollowPanel::LiveFollowPanel(QWidget *parent) : rviz_common::Panel(parent) {
  auto layout = new QVBoxLayout(this);
  physical_button_ = new QPushButton("Manipulador físico", this);
  physical_button_->setObjectName("toggle_physical");
  physical_button_->setCheckable(true);
  physical_button_->setEnabled(false);
  destination_ = new QLabel("Destino: Gazebo", this);
  destination_->setWordWrap(true);
  layout->addWidget(physical_button_);
  layout->addWidget(destination_);
  button_ = new QPushButton("Activar seguimiento", this);
  button_->setObjectName("toggle_follow");
  button_->setEnabled(false);
  status_ = new QLabel("Esperando el nodo de seguimiento…", this);
  status_->setWordWrap(true);
  auto hint = new QLabel("Arrastrá el marcador de seguimiento: cada cambio se "
                         "planifica y ejecuta automáticamente.",
                         this);
  hint->setWordWrap(true);
  grip_label_ = new QLabel("Apertura del gripper", this);
  grip_label_->hide();
  slider_ = new QSlider(Qt::Horizontal, this);
  slider_->setObjectName("gripper_target");
  slider_->setRange(0, 650);
  slider_->hide();
  slider_->setEnabled(false);
  layout->addWidget(button_);
  layout->addWidget(status_);
  layout->addWidget(grip_label_);
  layout->addWidget(slider_);
  layout->addWidget(hint);
  layout->addStretch();
  node_ = std::make_shared<rclcpp::Node>(
      "mycobot_live_panel", rclcpp::NodeOptions().use_global_arguments(false));
  heartbeat_ = node_->create_publisher<std_msgs::msg::Empty>(
      "/mycobot_live/heartbeat", 1);
  target_ = node_->create_publisher<std_msgs::msg::Float64>(
      "/mycobot_live/gripper_target", 1);
  client_ =
      node_->create_client<std_srvs::srv::SetBool>("/mycobot_live/set_enabled");
  auto qos = rclcpp::QoS(1).reliable().transient_local();
  selection_client_ = node_->create_client<std_srvs::srv::SetBool>(
      "/mycobot_control/select_physical");
  physical_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/mycobot_control/physical", qos,
      [this](std_msgs::msg::Bool::ConstSharedPtr m) {
        const bool changed = physical_ != m->data;
        physical_ = m->data;
        if (changed && getDisplayContext() &&
            getDisplayContext()->getRootDisplayGroup()) {
          auto group = getDisplayContext()->getRootDisplayGroup();
          for (int i = 0; i < group->numDisplays(); ++i) {
            auto display = group->getDisplayAt(i);
            if (display->getClassId() == "moveit_rviz_plugin/MotionPlanning")
              display->reset();
          }
        }
        have_coordinator_ = true;
        coordinator_age_.restart();
        QSignalBlocker block(physical_button_);
        physical_button_->setChecked(physical_);
      });
  destination_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/mycobot_control/status", qos,
      [this](std_msgs::msg::String::ConstSharedPtr m) {
        destination_->setText(QString::fromStdString(m->data));
      });
  connect(physical_button_, &QPushButton::clicked, this, [this] {
    {
      QSignalBlocker block(physical_button_);
      physical_button_->setChecked(physical_);
    }
    if (selecting_ || enabled_ || pending_ ||
        !selection_client_->service_is_ready())
      return;
    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
    req->data = !physical_;
    selection_future_ = selection_client_->async_send_request(req);
    selecting_ = true;
    selection_age_.restart();
    physical_button_->setEnabled(false);
    button_->setEnabled(false);
    destination_->setText("Cambiando destino…");
  });
  enabled_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/mycobot_live/enabled", qos,
      [this](std_msgs::msg::Bool::ConstSharedPtr m) {
        const bool activating = !enabled_ && m->data;
        enabled_ = m->data;
        if (activating && getDisplayContext()) {
          auto tools = getDisplayContext()->getToolManager();
          if (tools)
            for (int i = 0; i < tools->numTools(); ++i) {
              auto tool = tools->getTool(i);
              if (tool->getClassId() == "rviz_default_plugins/Interact") {
                tools->setCurrentTool(tool);
                break;
              }
            }
        }
        have_backend_ = true;
        backend_age_.restart();
        button_->setText(enabled_ ? "Detener seguimiento"
                                  : "Activar seguimiento");
        slider_->setEnabled(enabled_ && have_grip_ && !pending_);
      });
  variant_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/mycobot_live/has_gripper", qos,
      [this](std_msgs::msg::Bool::ConstSharedPtr m) {
        has_gripper_ = m->data;
        slider_->setVisible(has_gripper_);
        grip_label_->setVisible(has_gripper_);
      });
  status_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/mycobot_live/status", qos,
      [this](std_msgs::msg::String::ConstSharedPtr m) {
        status_->setText(QString::fromStdString(m->data));
      });
  grip_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
      "/mycobot_live/gripper_position", qos,
      [this](std_msgs::msg::Float64::ConstSharedPtr m) {
        if (!enabled_ || !have_grip_ || pending_) {
          QSignalBlocker block(slider_);
          slider_->setValue(qRound((m->data + .5) * 1000));
          have_grip_ = true;
        }
        slider_->setEnabled(enabled_ && have_grip_ && !pending_);
      });
  connect(slider_, &QSlider::valueChanged, this, [this](int value) {
    if (!enabled_ || pending_ || !have_grip_)
      return;
    std_msgs::msg::Float64 m;
    m.data = std::clamp(-.5 + value / 1000., -.5, .15);
    target_->publish(m);
  });
  connect(button_, &QPushButton::clicked, this, [this] {
    if (pending_ || !client_->service_is_ready())
      return;
    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
    req->data = !enabled_;
    future_ = client_->async_send_request(req);
    pending_ = true;
    request_age_.restart();
    button_->setEnabled(false);
    slider_->setEnabled(false);
    status_->setText("Cambiando modo…");
  });
  connect(&timer_, &QTimer::timeout, this, [this] { poll(); });
  timer_.start(100);
}
void LiveFollowPanel::poll() {
  if (selecting_ && selection_future_ &&
      selection_future_->wait_for(std::chrono::seconds(0)) ==
          std::future_status::ready) {
    auto response = selection_future_->get();
    destination_->setText(QString::fromStdString(response->message));
    selecting_ = false;
  } else if (selecting_ && selection_age_.elapsed() > 5000) {
    selection_client_->remove_pending_request(*selection_future_);
    selecting_ = false;
    destination_->setText("No se confirmó el destino; revisá la conexión");
  }
  if (!rclcpp::ok())
    return;
  rclcpp::spin_some(node_);
  updatePlanningMarker();
  heartbeat_->publish(std_msgs::msg::Empty());
  if (pending_ &&
      future_->wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    auto response = future_->get();
    pending_ = false;
    status_->setText(QString::fromStdString(response->message));
  } else if (pending_ && request_age_.elapsed() > 10000) {
    client_->remove_pending_request(*future_);
    pending_ = false;
    status_->setText(
        "No se confirmó el cambio. Revisá Gazebo y volvé a detener.");
  }
  button_->setEnabled(!selecting_ && !pending_ && have_backend_ &&
                      client_->service_is_ready());
  slider_->setEnabled(!selecting_ && !pending_ && enabled_ && have_grip_);
  physical_button_->setEnabled(
      have_coordinator_ && coordinator_age_.elapsed() < 1500 && !selecting_ &&
      !pending_ && !enabled_ && selection_client_->service_is_ready());
  if (have_backend_ && backend_age_.elapsed() > 1500) {
    button_->setEnabled(false);
    slider_->setEnabled(false);
    status_->setText("Sin conexión con el nodo de seguimiento");
  }
}
void LiveFollowPanel::updatePlanningMarker() {
  if (!enabled_) {
    planning_marker_visibility_.update(nullptr, false);
    return;
  }
  auto context = getDisplayContext();
  if (!context || !context->getRootDisplayGroup())
    return;
  auto group = context->getRootDisplayGroup();
  for (int i = 0; i < group->numDisplays(); ++i) {
    auto display = group->getDisplayAt(i);
    if (display->getClassId() != "moveit_rviz_plugin/MotionPlanning")
      continue;
    for (int j = 0; j < display->numChildren(); ++j) {
      auto property = display->childAt(j);
      if (property->getName() == "Planning Request")
        planning_marker_visibility_.update(property, true);
    }
  }
}
LiveFollowPanel::~LiveFollowPanel() { timer_.stop(); }
} // namespace mycobot_280_live_control
PLUGINLIB_EXPORT_CLASS(mycobot_280_live_control::LiveFollowPanel,
                       rviz_common::Panel)
