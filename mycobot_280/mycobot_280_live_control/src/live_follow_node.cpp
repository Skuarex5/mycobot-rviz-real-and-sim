#include "mycobot_280_live_control/control_session.hpp"
#include <action_msgs/msg/goal_status_array.hpp>
#include <algorithm>
#include <cmath>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <map>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/action/move_group.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
using namespace std::chrono_literals;
namespace mycobot_280_live_control {
static double steadyNow() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
class LiveFollowNode : public rclcpp::Node {
  using Move = moveit_msgs::action::MoveGroup;
  using Handle = rclcpp_action::ClientGoalHandle<Move>;

public:
  LiveFollowNode()
      : Node("mycobot_live_follow"),
        gripper_(declare_parameter<bool>("adaptive_gripper", false)) {
    for (const auto &p :
         get_node_parameters_interface()->get_parameter_overrides())
      if (!has_parameter(p.first))
        declare_parameter(p.first, p.second);
  }
  void initialize() {
    auto self = shared_from_this();
    monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
        self, "robot_description");
    if (!monitor_->getPlanningScene())
      throw std::runtime_error("No se pudo cargar el modelo");
    monitor_->startSceneMonitor("/monitored_planning_scene");
    monitor_->startStateMonitor("/joint_states");
    arm_names_ = monitor_->getRobotModel()
                     ->getJointModelGroup("arm_group")
                     ->getActiveJointModelNames();
    tool_ = gripper_ ? "gripper_tool_frame" : "joint6_flange";
    list_ = create_client<controller_manager_msgs::srv::ListControllers>(
        "/controller_manager/list_controllers");
    move_ = rclcpp_action::create_client<Move>(
        self, has_parameter("planning_action")
                  ? get_parameter("planning_action").as_string()
                  : declare_parameter<std::string>("planning_action",
                                                   "/move_action"));
    auto latched = rclcpp::QoS(1).reliable().transient_local();
    enabled_pub_ =
        create_publisher<std_msgs::msg::Bool>("/mycobot_live/enabled", latched);
    variant_pub_ = create_publisher<std_msgs::msg::Bool>(
        "/mycobot_live/has_gripper", latched);
    status_pub_ = create_publisher<std_msgs::msg::String>(
        "/mycobot_live/status", latched);
    grip_state_pub_ = create_publisher<std_msgs::msg::Float64>(
        "/mycobot_live/gripper_position", latched);
    heartbeat_sub_ = create_subscription<std_msgs::msg::Empty>(
        "/mycobot_live/heartbeat", 1,
        [this](std_msgs::msg::Empty::ConstSharedPtr) {
          std::lock_guard<std::mutex> lock(mutex_);
          heartbeat_ = steadyNow();
        });
    physical_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/mycobot_control/physical", latched,
        [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (active_ && physical_ != msg->data)
            stop("Destino cambiado; volvé a activar seguimiento");
          physical_ = msg->data;
        });
    joints_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          std::map<std::string, double> sample;
          for (size_t i = 0; i < msg->name.size() && i < msg->position.size();
               ++i)
            if (std::isfinite(msg->position[i]))
              sample[msg->name[i]] = msg->position[i];
          for (const auto &name : arm_names_)
            if (!sample.count(name))
              return;
          if (gripper_ && !sample.count("gripper_controller"))
            return;
          measured_ = std::move(sample);
          joints_time_ = steadyNow();
        });
    grip_sub_ = create_subscription<std_msgs::msg::Float64>(
        "/mycobot_live/gripper_target", 1,
        [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (active_ && gripper_ && ControlSession::validGripper(msg->data)) {
            grip_target_ = msg->data;
            grip_revision_ = ++revision_;
          }
        });
    // Also observe planning ownership: activation cannot take over a plan that
    // has not yet reached its trajectory controller.
    std::vector<std::string> actions = {
        "move_action", "mycobot_live/move_action", "execute_trajectory",
        "mycobot280m5_group_controller/follow_joint_trajectory"};
    if (gripper_)
      actions.push_back(
          "mycobot280m5_gripper_controller/follow_joint_trajectory");
    for (const auto &action : actions)
      action_subs_.push_back(
          create_subscription<action_msgs::msg::GoalStatusArray>(
              "/" + action + "/_action/status", latched,
              [this,
               action](action_msgs::msg::GoalStatusArray::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                busy_[action] =
                    std::any_of(msg->status_list.begin(),
                                msg->status_list.end(), [](const auto &s) {
                                  return s.status >= 1 && s.status <= 3;
                                });
              }));
    marker_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
        "/mycobot_live/marker", self);
    service_group_ =
        create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    service_ = create_service<std_srvs::srv::SetBool>(
        "/mycobot_live/set_enabled",
        [this](std_srvs::srv::SetBool::Request::SharedPtr req,
               std_srvs::srv::SetBool::Response::SharedPtr res) {
          std::unique_lock<std::mutex> transition(transition_,
                                                  std::try_to_lock);
          if (!transition.owns_lock()) {
            res->message = "Cambio de modo en curso";
            return;
          }
          if (req->data) {
            res->success = enable();
          } else {
            std::lock_guard<std::mutex> lock(mutex_);
            stop("Seguimiento desactivado");
            res->success = true;
          }
          std::lock_guard<std::mutex> lock(mutex_);
          res->message = status_;
        },
        rmw_qos_profile_services_default, service_group_);
    timer_ = create_wall_timer(40ms, [this] { tick(); });
    publishStatus("Seguimiento desactivado");
  }
  void shutdown() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (marker_)
      stop("Seguimiento desactivado");
  }

private:
  bool busy() const {
    return pending_ || std::any_of(busy_.begin(), busy_.end(),
                                   [](const auto &b) { return b.second; });
  }
  bool fresh() const {
    return steadyNow() - heartbeat_ < .75 &&
           steadyNow() - joints_time_ < (physical_ ? .8 : .5);
  }
  bool enable() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (active_)
        return true;
      if (!fresh() || busy() || !move_->action_server_is_ready()) {
        publishStatus("No se puede activar: revisá conexión, estado o "
                      "ejecución en curso");
        return false;
      }
    }
    // Live mode uses exactly the same trajectory controllers as Plan & Execute.
    // Do not switch to the old direct-position/Servo controllers.
    if (!list_->service_is_ready())
      return false;
    auto request = list_->async_send_request(
        std::make_shared<
            controller_manager_msgs::srv::ListControllers::Request>());
    if (request.wait_for(2s) != std::future_status::ready) {
      list_->remove_pending_request(request);
      std::lock_guard<std::mutex> lock(mutex_);
      publishStatus(
          "No se confirmó el estado de los controladores de trayectoria");
      return false;
    }
    auto controllers = request.get()->controller;
    std::vector<std::string> names = {"mycobot280m5_group_controller"};
    if (gripper_)
      names.push_back("mycobot280m5_gripper_controller");
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto &name : names) {
      if (std::none_of(controllers.begin(), controllers.end(),
                       [&](const auto &c) {
                         return c.name == name && c.state == "active";
                       })) {
        publishStatus(
            "Activá los controladores de trayectoria para usar seguimiento");
        return false;
      }
    }
    if (!fresh() || busy())
      return false;
    target_.header.frame_id = "g_base";
    target_.pose = measuredPose();
    arm_revision_ = grip_revision_ = 0;
    gesture_ = MarkerGesture{};
    active_ = true;
    createMarker();
    publishStatus("Seguimiento activo: Plan & Execute automático");
    return true;
  }
  geometry_msgs::msg::Pose measuredPose() {
    planning_scene_monitor::LockedPlanningSceneRO scene(monitor_);
    auto state = scene->getCurrentState();
    for (const auto &name : arm_names_)
      state.setVariablePosition(name, measured_.at(name));
    if (gripper_)
      state.setVariablePosition("gripper_controller",
                                measured_.at("gripper_controller"));
    state.update();
    return tf2::toMsg(state.getGlobalLinkTransform("g_base").inverse() *
                      state.getGlobalLinkTransform(tool_));
  }
  void publishStatus(const std::string &value) {
    status_ = value;
    std_msgs::msg::String s;
    s.data = value;
    status_pub_->publish(s);
    std_msgs::msg::Bool b;
    b.data = active_;
    enabled_pub_->publish(b);
    b.data = gripper_;
    variant_pub_->publish(b);
  }
  // Caller holds mutex_. Keep the action slot occupied until its terminal
  // result, including acceptance arriving after Stop. Never overlap sessions.
  void stop(const std::string &reason) {
    active_ = false;
    arm_revision_ = grip_revision_ = 0;
    gesture_ = MarkerGesture{};
    marker_->clear();
    marker_->applyChanges();
    if (handle_ && !cancel_sent_) {
      move_->async_cancel_goal(handle_);
      cancel_sent_ = true;
    }
    publishStatus(reason + (pending_ ? "; cancelando ejecución…" : ""));
  }
  void createMarker() {
    visualization_msgs::msg::InteractiveMarker m;
    m.header.frame_id = "g_base";
    m.name = "tool_target";
    m.description = "Seguimiento: Plan & Execute automático";
    m.pose = target_.pose;
    m.scale = .09;
    for (int axis = 0; axis < 3; ++axis) {
      visualization_msgs::msg::InteractiveMarkerControl c;
      c.orientation.w = axis == 0 ? 1. : std::sqrt(.5);
      if (axis == 1)
        c.orientation.z = std::sqrt(.5);
      if (axis == 2)
        c.orientation.y = -std::sqrt(.5);
      c.orientation_mode = c.FIXED;
      c.always_visible = true;
      c.name = "rotate_" + std::to_string(axis);
      c.description = std::string("Girar ") + "XYZ"[axis];
      c.interaction_mode = c.ROTATE_AXIS;
      m.controls.push_back(c);
      c.name = "move_" + std::to_string(axis);
      c.description = std::string("Mover ") + "XYZ"[axis];
      c.interaction_mode = c.MOVE_AXIS;
      m.controls.push_back(c);
    }
    marker_->insert(
        m,
        [this](
            visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr
                f) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (!active_ || f->header.frame_id != "g_base" ||
              !gesture_.accept(f->event_type, f->client_id, f->control_name))
            return;
          const auto &p = f->pose;
          Eigen::Quaterniond q(p.orientation.w, p.orientation.x,
                               p.orientation.y, p.orientation.z);
          if (!std::isfinite(p.position.x) || !std::isfinite(p.position.y) ||
              !std::isfinite(p.position.z) || !q.coeffs().allFinite() ||
              q.norm() < .001)
            return;
          q.normalize();
          target_.pose = p;
          target_.pose.orientation = tf2::toMsg(q);
          arm_revision_ = ++revision_;
        });
    marker_->applyChanges();
  }
  Move::Goal goal(bool arm) {
    Move::Goal g;
    auto &r = g.request;
    r.group_name = arm ? "arm_group" : "gripper_group";
    r.num_planning_attempts = 3;
    r.allowed_planning_time = 2.;
    // The coordinator applies additional physical destination limits.
    r.max_velocity_scaling_factor = .25;
    r.max_acceleration_scaling_factor = .25;
    r.start_state.is_diff = true;
    g.planning_options.planning_scene_diff.is_diff = true;
    g.planning_options.planning_scene_diff.robot_state.is_diff = true;
    g.planning_options.plan_only = false;
    if (arm) {
      target_.header.stamp = now();
      r.goal_constraints.push_back(
          kinematic_constraints::constructGoalConstraints(tool_, target_, .001,
                                                          .01));
    } else {
      moveit_msgs::msg::Constraints c;
      moveit_msgs::msg::JointConstraint j;
      j.joint_name = "gripper_controller";
      j.position = grip_target_;
      j.tolerance_above = j.tolerance_below = .002;
      j.weight = 1.;
      c.joint_constraints.push_back(j);
      r.goal_constraints.push_back(c);
    }
    return g;
  }
  void tick() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (gripper_ && measured_.count("gripper_controller")) {
      std_msgs::msg::Float64 p;
      p.data = measured_.at("gripper_controller");
      grip_state_pub_->publish(p);
    }
    if (active_ && !fresh())
      stop("Conexión o mediciones perdidas; seguimiento detenido");
    if (active_ && pending_ && steadyNow() - sent_ > 180.)
      stop("Ejecución sin respuesta; seguimiento detenido");
    if (++ticks_ % 10 == 0)
      publishStatus(status_);
    // Rate-limit dispatch, not incoming feedback: continuous dragging must
    // start a plan without waiting for the mouse to stop. During execution,
    // keep only the latest pending target for each group.
    if (!active_ || busy() || (!arm_revision_ && !grip_revision_) ||
        steadyNow() - sent_ < .12)
      return;
    if (!move_->action_server_is_ready()) {
      stop("MoveIt no disponible");
      return;
    }
    bool arm =
        arm_revision_ && (!grip_revision_ || arm_revision_ < grip_revision_);
    auto request = goal(arm);
    if (arm)
      arm_revision_ = 0;
    else
      grip_revision_ = 0;
    pending_ = true;
    cancel_sent_ = false;
    sent_ = steadyNow();
    publishStatus(arm ? "Planificando y ejecutando brazo…"
                      : "Planificando y ejecutando pinza…");
    rclcpp_action::Client<Move>::SendGoalOptions options;
    options.goal_response_callback = [this](Handle::SharedPtr handle) {
      std::lock_guard<std::mutex> lock(mutex_);
      handle_ = handle;
      if (!handle) {
        pending_ = false;
        if (active_)
          publishStatus("MoveIt rechazó el movimiento; elegí otro objetivo");
      } else if (!active_) {
        move_->async_cancel_goal(handle_);
        cancel_sent_ = true;
      }
    };
    options.result_callback = [this](const Handle::WrappedResult &result) {
      std::lock_guard<std::mutex> lock(mutex_);
      pending_ = false;
      handle_.reset();
      cancel_sent_ = false;
      if (active_) {
        bool ok = result.code == rclcpp_action::ResultCode::SUCCEEDED &&
                  result.result &&
                  result.result->error_code.val ==
                      moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
        publishStatus(
            ok ? "Seguimiento activo: movimiento ejecutado"
               : "No se pudo planificar o ejecutar; elegí otro objetivo");
      } else {
        publishStatus("Seguimiento detenido; ejecución finalizada");
      }
    };
    move_->async_send_goal(request, options);
  }
  bool gripper_, physical_ = false, active_ = false, pending_ = false,
                 cancel_sent_ = false;
  int ticks_ = 0;
  uint64_t revision_ = 0, arm_revision_ = 0, grip_revision_ = 0;
  double heartbeat_ = 0, joints_time_ = 0, sent_ = 0, grip_target_ = 0;
  std::string tool_, status_;
  std::mutex mutex_, transition_;
  std::vector<std::string> arm_names_;
  std::map<std::string, double> measured_;
  std::map<std::string, bool> busy_;
  MarkerGesture gesture_;
  geometry_msgs::msg::PoseStamped target_;
  planning_scene_monitor::PlanningSceneMonitorPtr monitor_;
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> marker_;
  rclcpp_action::Client<Move>::SharedPtr move_;
  Handle::SharedPtr handle_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr
      list_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enabled_pub_, variant_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr grip_state_pub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr heartbeat_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr physical_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr grip_sub_;
  std::vector<
      rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr>
      action_subs_;
  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace mycobot_280_live_control
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mycobot_280_live_control::LiveFollowNode>();
  node->initialize();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                    4);
  executor.add_node(node);
  executor.spin();
  node->shutdown();
  rclcpp::shutdown();
  return 0;
}
