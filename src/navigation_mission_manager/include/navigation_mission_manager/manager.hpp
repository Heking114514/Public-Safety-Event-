#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <mission_control_interfaces/action/report_event.hpp>
#include <mission_control_interfaces/msg/event_candidate.hpp>
#include <mission_control_interfaces/msg/mission_state.hpp>
#include <mission_control_interfaces/msg/motion_hold_state.hpp>
#include <mission_control_interfaces/msg/navigation_recovery_state.hpp>
#include <mission_control_interfaces/srv/continue_retreat.hpp>
#include <mission_control_interfaces/srv/control_mission.hpp>
#include <mission_control_interfaces/srv/confirm_rfid_checkpoint.hpp>
#include <mission_control_interfaces/srv/correct_map_position.hpp>
#include <mission_control_interfaces/srv/set_motion_hold.hpp>
#include <mission_control_interfaces/srv/speak_text.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <new_map_planner/msg/route_plan.hpp>
#include <new_map_planner/srv/plan_route.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int64.hpp>

#include "navigation_mission_manager/events.hpp"
#include "navigation_mission_manager/mission.hpp"
#include <mission_control_interfaces/srv/select_recovery.hpp>
#include "navigation_mission_manager/recovery_session.hpp"
#include "navigation_mission_manager/obstacles.hpp"

namespace navigation_mission_manager
{

double steady_seconds();
std::vector<std::string> parse_ids(const std::string & text);
std::optional<double> quaternion_yaw(const geometry_msgs::msg::Quaternion & quaternion);

struct VehiclePose
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double linear_speed{0.0};
  double received_at{0.0};

  bool fresh(double timeout_s) const;
};

struct ActiveRoute
{
  uint64_t route_id{0};
  PlanSpec spec;
  new_map_planner::msg::RoutePlan plan;
  bool acknowledged{false};
  bool motion_seen{false};
};

struct PlanningContext
{
  uint8_t start_mode{new_map_planner::srv::PlanRoute::Request::START_MODE_NORMAL};
  std::string recovery_anchor_node_id;
  std::string recovery_incoming_lane_id;
  bool allow_reverse_departure{false};
  bool has_safe_pose{false};
  geometry_msgs::msg::Pose safe_pose;
};


class MissionManager : public rclcpp::Node
{
public:
  using ReportEvent = mission_control_interfaces::action::ReportEvent;
  using ReportGoalHandle = rclcpp_action::ClientGoalHandle<ReportEvent>;

  MissionManager();
  void request_shutdown_hold();

private:
  friend struct ManagerRecoveryTestAccess;
  friend struct ManagerRfidTestAccess;
  void declare_parameters();
  void load_configuration();
  void create_interfaces();
  void tick();
  bool ready() const;

  void set_state(uint8_t state, const std::string & detail);
  mission_control_interfaces::msg::MissionState state_message() const;
  void publish_state();
  void publish_text(const std::string & text);
  void request_hold(const std::string & suffix, bool hold, const std::string & reason);
  void on_hold_response(
    rclcpp::Client<mission_control_interfaces::srv::SetMotionHold>::SharedFuture future);
  void on_hold_state(
    const mission_control_interfaces::msg::MotionHoldState::SharedPtr message);

  void on_control(
    const mission_control_interfaces::srv::ControlMission::Request::SharedPtr request,
    mission_control_interfaces::srv::ControlMission::Response::SharedPtr response);
  void on_public_motion_hold(
    const mission_control_interfaces::srv::SetMotionHold::Request::SharedPtr request,
    mission_control_interfaces::srv::SetMotionHold::Response::SharedPtr response);
  void on_public_continue_retreat(
    const mission_control_interfaces::srv::ContinueRetreat::Request::SharedPtr request,
    mission_control_interfaces::srv::ContinueRetreat::Response::SharedPtr response);

  void start_mission(const std::vector<std::string> & targets);
  void abort_mission();
  void reset_mission();
  void schedule_plan(bool replanning);
  void request_plan();
  void on_plan_response(
    uint64_t generation, const PlanSpec & spec,
    const PlanningContext & context,
    rclcpp::Client<new_map_planner::srv::PlanRoute>::SharedFuture future);
  void plan_failed(const std::string & message);
  void on_route_ack(const std_msgs::msg::UInt64::SharedPtr message);
  void accept_ack(uint64_t route_id);
  void handle_goal_reached();
  void finish_mission();
  void clear_active_route();
  void fault(const std::string & message);

  void on_navigator_status(const std_msgs::msg::String::SharedPtr message);
  void on_navigation_recovery_state(
    const mission_control_interfaces::msg::NavigationRecoveryState::SharedPtr message);
  void handle_turn_replan();
  std::optional<std::pair<std::string, std::string>> turn_recovery_context() const;
  void handle_obstacle_replan();
  void handle_plan_rejection(const std::string & message);
  void request_additional_retreat(const std::string & planner_message);
  void on_retreat_response(
    uint64_t generation, const std::string & planner_message,
    rclcpp::Client<mission_control_interfaces::srv::ContinueRetreat>::SharedFuture future);
  void clear_obstacle_recovery();
  void on_pose_recovery(const mission_control_interfaces::msg::NavigationRecoveryState & state);
  void service_pose_recovery();
  void clear_pose_recovery();
  bool waiting_for_pose_candidates() const
  {
    return pose_recovery_enabled_ && latest_navigation_recovery_.has_value() &&
      latest_navigation_recovery_->phase == "braking" && active_route_.has_value() &&
      latest_navigation_recovery_->route_id == active_route_->route_id;
  }
  void preserve_interrupted_progress();
  void evaluate_recovery_candidate();
  void on_recovery_evaluation(
    uint64_t generation, uint64_t revision, uint32_t candidate_id, const PlanSpec & spec,
    rclcpp::Client<new_map_planner::srv::PlanRoute>::SharedFuture future);
  void select_recovery_candidate(uint32_t candidate_id);
  void on_recovery_selection(
    uint64_t generation, uint64_t revision, uint32_t candidate_id,
    rclcpp::Client<mission_control_interfaces::srv::SelectRecovery>::SharedFuture future);


  void on_odometry(const nav_msgs::msg::Odometry::SharedPtr message);
  void on_insight(const std_msgs::msg::String::SharedPtr message);
  void on_safe(const std_msgs::msg::String::SharedPtr message);
  void record_visual_obstacle(double distance);
  void on_front_range(const sensor_msgs::msg::Range::SharedPtr message);
  void on_front_blocked(const std_msgs::msg::Bool::SharedPtr message);
  void on_obstacle_polygon(const geometry_msgs::msg::PolygonStamped::SharedPtr message);
  bool promote_pending_obstacle();
  void expire_dynamic_obstacles(double now);
  std::optional<std::string> infer_blocked_lane_id(const Polygon2d & polygon) const;
  void on_event_candidate(
    const mission_control_interfaces::msg::EventCandidate::SharedPtr message);
  void service_event_reporting(double now);
  void on_report_goal(uint64_t generation, ReportGoalHandle::SharedPtr handle);
  void on_report_result(uint64_t generation, const ReportGoalHandle::WrappedResult & wrapped);
  void report_failed(const std::string & message);
  void cancel_report();

  void reset_rfid();
  void on_rfid_number(const std_msgs::msg::Int32::SharedPtr message);
  void enqueue_speech_text(const std::string & text, bool request_hold);
  void on_speech_text(const std_msgs::msg::String::SharedPtr message);
  void service_speech_text(double now);
  void service_rfid(double now);
  void validate_pending_rfid(double now);
  void reject_pending_rfid(const std::string & reason);
  void send_rfid_speech(double now);
  void on_rfid_speech(
    int number,
    rclcpp::Client<mission_control_interfaces::srv::SpeakText>::SharedFuture future);
  void on_rfid_correction(
    int number,
    rclcpp::Client<mission_control_interfaces::srv::CorrectMapPosition>::SharedFuture future);
  void on_rfid_checkpoint_authorization(
    uint64_t route_id, std::size_t waypoint_index, bool route_mode,
    rclcpp::Client<mission_control_interfaces::srv::ConfirmRfidCheckpoint>::SharedFuture future);
  void authorize_route_checkpoint(std::size_t waypoint_index);
  std::optional<std::pair<std::size_t, std::string>> expected_rfid_checkpoint() const;
  void on_current_waypoint(const std_msgs::msg::Int32::SharedPtr message);
  void begin_rfid_post_speech(double now, bool speech_required);
  void finish_rfid_workflow();
  void refresh_rfid_duplicate_guard();

  geometry_msgs::msg::Pose pose_message() const;
  geometry_msgs::msg::Polygon polygon_message(const Polygon2d & polygon) const;
  std::optional<EventData> event_data(
    const mission_control_interfaces::msg::EventCandidate & message) const;
  mission_control_interfaces::msg::EventCandidate event_message(
    const EventData & event) const;

  std::unique_ptr<StagedMission> mission_;
  std::unique_ptr<EventRegistry> events_;
  std::vector<std::string> default_targets_;
  std::string task_confirmation_mode_;

  uint8_t state_{mission_control_interfaces::msg::MissionState::WAIT_READY};
  std::string detail_{"等待规划器、控制器和融合里程计"};
  std::optional<VehiclePose> pose_;
  int64_t last_odom_stamp_ns_{0};
  mutable std::mutex hold_state_mutex_;
  bool motion_held_{false};
  std::set<std::string> active_hold_sources_;
  std::atomic<bool> mission_running_public_{false};
  std::string navigator_status_{"UNKNOWN"};
  std::optional<ActiveRoute> active_route_;
  std::set<uint64_t> ack_cache_;
  double ack_deadline_{0.0};
  bool planning_pending_{false};
  uint64_t pending_plan_generation_{0};
  uint64_t plan_generation_{0};
  std::optional<double> plan_due_at_;
  int plan_retries_{0};
  int no_progress_routes_{0};
  bool replanning_{false};
  PlanningContext next_planning_context_{};
  std::optional<mission_control_interfaces::msg::NavigationRecoveryState>
    latest_navigation_recovery_;
  bool turn_replan_waiting_for_recovery_{false};
  bool pose_recovery_enabled_{false};
  RecoverySession pose_recovery_;
  std::vector<mission_control_interfaces::msg::RecoveryCandidate> recovery_candidates_;
  std::size_t recovery_candidate_index_{0};
  bool recovery_evaluation_pending_{false};
  bool recovery_selection_pending_{false};
  bool recovery_selection_accepted_{false};
  bool recovery_handed_off_{false};
  uint64_t recovery_snapshot_revision_{0};
  bool pose_obstacles_latched_{false};
  int completed_stages_{0};

  bool retreat_pending_{false};
  uint64_t retreat_generation_{0};
  int retreat_extensions_{0};
  bool awaiting_retreat_{false};
  bool obstacle_recovery_active_{false};
  bool route_transition_recovery_active_{false};
  bool completed_route_retreat_available_{false};
  bool awaiting_heading_retreat_{false};
  uint8_t ordinary_nodes_since_task_{1};

  bool front_blocked_{false};
  std::set<std::string> visual_announced_targets_;
  std::optional<std::pair<double, double>> latest_range_;
  std::optional<std::pair<Polygon2d, double>> pending_obstacle_;
  std::vector<Polygon2d> dynamic_obstacles_;
  std::optional<double> dynamic_obstacles_updated_at_;
  std::set<std::string> blocked_lane_ids_;
  uint64_t obstacle_revision_{0};

  bool report_goal_pending_{false};
  bool report_result_pending_{false};
  ReportGoalHandle::SharedPtr report_goal_handle_;
  uint64_t report_generation_{0};
  std::optional<double> report_wait_started_;
  bool report_hold_requested_{false};

  std::map<std::size_t, double> rfid_recent_;
  std::set<std::size_t> rfid_confirmed_waypoints_;
  std::set<std::size_t> route_checkpoint_authorizations_pending_;
  std::set<std::string> rfid_confirmed_tasks_;
  std::set<int> rfid_announced_;
  struct CompletedRfidVisit
  {
    int card;
    double x;
    double y;
  };
  std::optional<CompletedRfidVisit> rfid_duplicate_guard_;
  std::optional<int> rfid_pending_card_;
  uint64_t rfid_pending_route_id_{0};
  double rfid_next_hold_retry_at_{0.0};
  std::optional<int> rfid_active_;
  int rfid_active_card_{0};
  std::string rfid_active_supernode_;
  std::size_t rfid_active_waypoint_index_{0};
  uint64_t rfid_active_route_id_{0};
  int32_t current_waypoint_index_{0};
  std::string rfid_phase_{"idle"};
  std::optional<double> rfid_release_at_;
  std::optional<double> rfid_correction_stable_since_;
  std::optional<double> rfid_stationary_since_;
  double rfid_stationary_reference_x_{0.0};
  double rfid_stationary_reference_y_{0.0};
  double rfid_correction_deadline_{0.0};
  bool rfid_speech_required_{false};
  bool rfid_skip_correction_{false};
  bool rfid_speech_pending_{false};
  bool rfid_correction_pending_{false};
  bool rfid_authorization_pending_{false};
  int rfid_speech_attempts_{0};
  double rfid_next_speech_at_{0.0};
  struct SpeechItem
  {
    std::string text;
    bool request_hold{true};
  };
  std::deque<SpeechItem> speech_queue_;
  std::optional<SpeechItem> speech_active_;
  std::optional<double> speech_release_at_;
  bool speech_pending_{false};
  double speech_next_send_at_{0.0};

  double speech_hold_duration_s_{3.0};
  double speech_min_interval_s_{1.2};
  double rfid_event_cooldown_s_{3.0};
  double rfid_hold_duration_s_{3.0};
  double rfid_stop_duration_s_{1.0};
  double rfid_detection_radius_m_{0.30};
  double rfid_correction_tolerance_m_{0.025};
  double rfid_correction_settle_s_{0.20};
  double rfid_correction_timeout_s_{3.0};
  double rfid_scan_max_distance_m_{0.20};
  double rfid_stop_speed_mps_{0.02};
  double rfid_stationary_position_tolerance_m_{0.005};
  double rfid_stationary_settle_s_{0.20};
  std::string rfid_announcement_template_;
  int rfid_max_speech_retries_{2};
  double odom_timeout_s_{0.6};
  double route_ack_timeout_s_{3.0};
  double replan_delay_s_{0.15};
  double plan_retry_delay_s_{1.0};
  int max_plan_retries_{3};
  double retreat_step_m_{0.4};
  int max_retreat_extensions_{4};
  int max_no_progress_routes_{2};
  double obstacle_sensor_forward_offset_m_{0.055};
  double obstacle_observation_timeout_s_{5.0};
  double obstacle_range_sync_s_{0.5};
  double obstacle_width_m_{0.24};
  double obstacle_depth_m_{0.12};
  double obstacle_merge_distance_m_{0.18};
  int max_dynamic_obstacles_{20};
  double dynamic_obstacle_retention_s_{120.0};
  bool report_requires_hold_{true};
  double report_server_timeout_s_{5.0};
  int max_report_retries_{2};
  double public_service_timeout_s_{2.0};
  bool auto_start_pending_{false};

  rclcpp::Client<mission_control_interfaces::srv::SelectRecovery>::SharedPtr select_recovery_client_;
  rclcpp::Client<new_map_planner::srv::PlanRoute>::SharedPtr planner_client_;
  rclcpp::Client<mission_control_interfaces::srv::SetMotionHold>::SharedPtr hold_client_;
  rclcpp::Client<mission_control_interfaces::srv::ContinueRetreat>::SharedPtr retreat_client_;
  rclcpp::Client<mission_control_interfaces::srv::ContinueRetreat>::SharedPtr
    route_retreat_client_;
  rclcpp_action::Client<ReportEvent>::SharedPtr reporter_client_;
  rclcpp::Client<mission_control_interfaces::srv::SpeakText>::SharedPtr speaker_client_;
  rclcpp::Client<mission_control_interfaces::srv::CorrectMapPosition>::SharedPtr
    position_corrector_client_;
  rclcpp::Client<mission_control_interfaces::srv::ConfirmRfidCheckpoint>::SharedPtr
    rfid_checkpoint_client_;

  rclcpp::Publisher<mission_control_interfaces::msg::MissionState>::SharedPtr state_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr text_publisher_;
  rclcpp::Publisher<mission_control_interfaces::msg::EventCandidate>::SharedPtr
    reported_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr route_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr visual_blocked_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr visual_range_publisher_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr navigator_subscription_;
  rclcpp::Subscription<mission_control_interfaces::msg::NavigationRecoveryState>::SharedPtr
    navigation_recovery_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr ack_subscription_;
  rclcpp::Subscription<mission_control_interfaces::msg::MotionHoldState>::SharedPtr
    hold_state_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr
    obstacle_polygon_subscription_;
  rclcpp::Subscription<mission_control_interfaces::msg::EventCandidate>::SharedPtr
    event_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr rfid_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr current_waypoint_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr speech_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr insight_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr safe_subscription_;

  rclcpp::Service<mission_control_interfaces::srv::ControlMission>::SharedPtr control_service_;
  rclcpp::Service<mission_control_interfaces::srv::SetMotionHold>::SharedPtr
    public_hold_service_;
  rclcpp::Service<mission_control_interfaces::srv::ContinueRetreat>::SharedPtr
    public_retreat_service_;
  rclcpp::CallbackGroup::SharedPtr public_service_group_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace navigation_mission_manager
