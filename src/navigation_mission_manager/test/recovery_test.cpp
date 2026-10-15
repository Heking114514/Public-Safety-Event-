#include <future>
#include <memory>

#include <gtest/gtest.h>
#include "navigation_mission_manager/manager.hpp"

namespace navigation_mission_manager
{
struct ManagerRecoveryTestAccess
{
  static void prepare(MissionManager & m)
  {
    m.pose_recovery_enabled_ = true;
    m.select_recovery_client_ = m.create_client<mission_control_interfaces::srv::SelectRecovery>(
      "/manager_test/select_recovery");
    m.mission_->start({"1", "2"});
    ActiveRoute route;
    route.route_id = 77;
    route.acknowledged = true;
    route.motion_seen = true;
    route.spec = m.mission_->plan_spec();
    route.plan.covered_lane_ids = {"ROAD_TOP_LEFT", "ROAD_TOP_RIGHT"};
    for (int i = 0; i < 3; ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = static_cast<double>(i);
      pose.pose.orientation.w = 1.0;
      route.plan.navigation_path.poses.push_back(pose);
    }
    route.plan.waypoint_supernode_ids = {"", "s1", "s2"};
    new_map_planner::msg::Maneuver drive;
    drive.type = new_map_planner::msg::Maneuver::DRIVE;
    drive.incoming_lane_id = "ROAD_TOP_LEFT";
    drive.end_pose = route.plan.navigation_path.poses[1].pose;
    route.plan.maneuvers.push_back(drive);
    drive.incoming_lane_id = "ROAD_TOP_RIGHT";
    drive.end_pose = route.plan.navigation_path.poses[2].pose;
    route.plan.maneuvers.push_back(drive);
    m.active_route_ = route;
    m.current_waypoint_index_ = 2;
    m.rfid_confirmed_waypoints_.insert(1);
    m.rfid_confirmed_tasks_.insert("1");
  }

  static mission_control_interfaces::msg::NavigationRecoveryState state()
  {
    mission_control_interfaces::msg::NavigationRecoveryState s;
    s.route_id = 77;
    s.recovery_id = 4;
    s.obstacle_revision = 9;
    s.phase = "braking";
    s.blocked_lane_id = "ROAD_TOP_RIGHT";
    for (const auto & xy : std::vector<std::pair<float, float>>{
      {2.0F, 0.0F}, {2.1F, 0.0F}, {2.1F, 0.1F}, {2.0F, 0.1F}})
    {
      geometry_msgs::msg::Point32 p;
      p.x = xy.first;
      p.y = xy.second;
      s.obstacle.points.push_back(p);
    }
    return s;
  }

  static mission_control_interfaces::msg::RecoveryCandidate candidate(uint32_t id)
  {
    mission_control_interfaces::msg::RecoveryCandidate c;
    c.candidate_id = id;
    c.exit_pose.position.x = 0.5;
    c.exit_pose.orientation.w = 1.0;
    c.resume_node = "test_node";
    c.resume_lane = "ROAD_TOP_LEFT";
    c.motion_cost = static_cast<double>(id);
    return c;
  }

  static void latch(MissionManager & m)
  {
    auto s = state();
    s.phase = "candidates";
    s.candidates = {candidate(1)};
    m.on_pose_recovery(s);
  }
  static bool active(const MissionManager & m) {return m.pose_recovery_.active;}
  static bool faulted(const MissionManager & m) {
    return m.state_ == mission_control_interfaces::msg::MissionState::FAULT;
  }
  static void message(MissionManager & m,
    const mission_control_interfaces::msg::NavigationRecoveryState & s) {m.on_pose_recovery(s);}

  static void check_latch(MissionManager & m)
  {
    EXPECT_FALSE(m.active_route_.has_value());
    EXPECT_EQ(m.mission_->completed_tasks().count("1"), 1U);
    EXPECT_EQ(m.mission_->completed_tasks().count("2"), 0U);
    EXPECT_EQ(m.mission_->confirmed_lanes().count("ROAD_TOP_LEFT"), 1U);
    EXPECT_EQ(m.mission_->confirmed_lanes().count("ROAD_TOP_RIGHT"), 0U);
    EXPECT_EQ(m.mission_->skipped_roads().count("ROAD_TOP_RIGHT"), 1U);
    const auto revision = m.obstacle_revision_;
    auto s = state();
    s.phase = "candidates";
    auto blocked = candidate(1);
    blocked.borrowed_lane_ids = {"ROAD_TOP_RIGHT"};
    s.candidates = {blocked, candidate(2)};
    m.recovery_candidates_.clear();
    m.on_pose_recovery(s);
    m.on_pose_recovery(s);
    EXPECT_EQ(m.obstacle_revision_, revision);
    EXPECT_EQ(m.dynamic_obstacles_.size(), 1U);
    ASSERT_EQ(m.recovery_candidates_.size(), 1U);
    EXPECT_EQ(m.recovery_candidates_.front().candidate_id, 2U);
    m.expire_dynamic_obstacles(steady_seconds() + 1000.0);
    EXPECT_EQ(m.dynamic_obstacles_.size(), 1U);
    EXPECT_EQ(m.blocked_lane_ids_.count("ROAD_TOP_RIGHT"), 1U);
  }

  static void complete(MissionManager & m)
  {
    m.recovery_selection_accepted_ = true;
    auto s = state();
    s.phase = "complete";
    s.valid = true;
    s.safe_pose.header.frame_id = "map";
    s.safe_pose.pose.position.x = 0.75;
    s.safe_pose.pose.position.y = -0.4;
    s.safe_pose.pose.orientation.w = 1.0;
    s.resume_node = "actual_node";
    s.resume_lane = "ROAD_TOP_LEFT";
    m.on_pose_recovery(s);
    EXPECT_TRUE(m.recovery_handed_off_);
    EXPECT_TRUE(m.next_planning_context_.has_safe_pose);
    EXPECT_DOUBLE_EQ(m.next_planning_context_.safe_pose.position.x, 0.75);
    EXPECT_EQ(m.next_planning_context_.start_mode,
      new_map_planner::srv::PlanRoute::Request::START_MODE_NORMAL);
    EXPECT_EQ(m.ordinary_nodes_since_task_, 0U);
    EXPECT_FALSE(m.obstacle_recovery_active_);
    EXPECT_FALSE(m.route_transition_recovery_active_);
    EXPECT_EQ(m.retreat_extensions_, 0);
    EXPECT_TRUE(m.plan_due_at_.has_value());
    const auto generation = m.plan_generation_;
    m.on_pose_recovery(s);
    EXPECT_EQ(m.plan_generation_, generation);
  }

  static void stale_after_abort(MissionManager & m)
  {
    const auto generation = m.pose_recovery_.generation;
    const auto revision = m.obstacle_revision_;
    m.abort_mission();
    EXPECT_FALSE(m.pose_recovery_.active);
    m.on_pose_recovery(state());
    EXPECT_FALSE(m.pose_recovery_.active);
    rclcpp::Client<mission_control_interfaces::srv::SelectRecovery>::SharedFuture invalid;
    m.on_recovery_selection(generation, revision, 1, invalid);
    EXPECT_FALSE(m.recovery_selection_accepted_);
    EXPECT_EQ(m.state_, mission_control_interfaces::msg::MissionState::IDLE);
    m.reset_mission();
    m.on_pose_recovery(state());
    EXPECT_FALSE(m.pose_recovery_.active);
  }

  static void handoff_response(MissionManager & m, bool success)
  {
    auto response = std::make_shared<new_map_planner::srv::PlanRoute::Response>();
    response->success = success;
    response->message = "handoff regression";
    auto & plan = response->plan;
    plan.plan_id = 78;
    plan.collision_checked = true;
    plan.visit_order = {"2"};
    plan.blocked_lane_ids_used.assign(m.blocked_lane_ids_.begin(), m.blocked_lane_ids_.end());
    plan.obstacle_revision = m.obstacle_revision_;
    geometry_msgs::msg::PoseStamped pose;
    pose.pose = m.next_planning_context_.safe_pose;
    plan.navigation_path.poses.push_back(pose);
    pose.pose.position.x = 1.8;
    pose.pose.position.y = -1.5;
    plan.navigation_path.poses.push_back(pose);
    plan.waypoint_supernode_ids = {"", "s2"};
    std::promise<new_map_planner::srv::PlanRoute::Response::SharedPtr> promise;
    auto future = promise.get_future().share();
    promise.set_value(response);
    m.on_plan_response(m.plan_generation_, m.mission_->plan_spec(),
      m.next_planning_context_, future);
  }

  static void handoff_lifecycle(MissionManager & m)
  {
    handoff_response(m, false);
    ASSERT_TRUE(m.recovery_handed_off_);
    ASSERT_TRUE(m.next_planning_context_.has_safe_pose);
    EXPECT_DOUBLE_EQ(m.next_planning_context_.safe_pose.position.x, 0.75);
    handoff_response(m, true);
    ASSERT_TRUE(m.active_route_.has_value());
    EXPECT_FALSE(m.next_planning_context_.has_safe_pose);
    EXPECT_FALSE(m.recovery_handed_off_);
    EXPECT_FALSE(m.pose_recovery_.active);
    m.active_route_->acknowledged = true;
    m.active_route_->motion_seen = true;
    m.pose_ = VehiclePose{1.8, -1.5, 3.14, 0.0, steady_seconds()};
    m.rfid_confirmed_tasks_.insert("2");
    m.handle_goal_reached();
    EXPECT_EQ(m.mission_->completed_tasks().count("2"), 1U);
    EXPECT_FALSE(m.active_route_.has_value());
    EXPECT_TRUE(m.plan_due_at_.has_value());
    EXPECT_FALSE(m.next_planning_context_.has_safe_pose);
    EXPECT_DOUBLE_EQ(m.pose_message().position.x, 1.8);
    EXPECT_DOUBLE_EQ(m.pose_message().position.y, -1.5);
  }

  static void rejection_after_goal(MissionManager & m)
  {
    m.completed_route_retreat_available_ = true;
    m.handle_plan_rejection("no forward route");
    EXPECT_FALSE(m.retreat_pending_);
    EXPECT_FALSE(m.route_transition_recovery_active_);
    EXPECT_EQ(m.plan_retries_, 1);
    EXPECT_EQ(m.state_, mission_control_interfaces::msg::MissionState::REPLANNING);
  }

  static void heading_fault(MissionManager & m)
  {
    auto status = std::make_shared<std_msgs::msg::String>();
    status->data = "FAULT_ROUTE_HEADING";
    m.on_navigator_status(status);
    EXPECT_TRUE(faulted(m));
    EXPECT_FALSE(m.awaiting_heading_retreat_);
  }

  static void selection_retry(MissionManager & m)
  {
    m.recovery_candidates_ = {candidate(1), candidate(2)};
    m.recovery_selection_pending_ = true;
    std::promise<mission_control_interfaces::srv::SelectRecovery::Response::SharedPtr> promise;
    auto future = promise.get_future().share();
    auto response = std::make_shared<mission_control_interfaces::srv::SelectRecovery::Response>();
    response->success = false;
    response->message = "local plan rejected";
    promise.set_value(response);
    m.on_recovery_selection(m.pose_recovery_.generation, m.obstacle_revision_, 1, future);
    EXPECT_EQ(m.recovery_candidate_index_, 1U);
    EXPECT_FALSE(m.recovery_selection_pending_);
    EXPECT_TRUE(m.pose_recovery_.active);
  }
  static void speech_failure(MissionManager & m)
  {
    m.rfid_active_ = 1;
    m.rfid_active_card_ = 1;
    m.rfid_active_supernode_ = "s1";
    m.rfid_speech_attempts_ = m.rfid_max_speech_retries_ + 1;
    m.rfid_phase_ = "announcing";
    std::promise<mission_control_interfaces::srv::SpeakText::Response::SharedPtr> promise;
    auto future = promise.get_future().share();
    auto response = std::make_shared<mission_control_interfaces::srv::SpeakText::Response>();
    response->success = false;
    response->message = "test speaker failure";
    promise.set_value(response);
    m.on_rfid_speech(1,future);
    EXPECT_EQ(m.state_, mission_control_interfaces::msg::MissionState::FAULT);
    EXPECT_EQ(m.rfid_announced_.count(1),0U);
    EXPECT_EQ(m.mission_->completed_tasks().count("1"),0U);
    EXPECT_NE(m.rfid_phase_,"post_speech");
  }
};
}  // namespace navigation_mission_manager

namespace nmm = navigation_mission_manager;
class ManagerRecoveryTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override
  {
    manager = std::make_shared<nmm::MissionManager>();
    nmm::ManagerRecoveryTestAccess::prepare(*manager);
  }
  std::shared_ptr<nmm::MissionManager> manager;
};

TEST_F(ManagerRecoveryTest, LatchesOncePreservesOnlyExecutedProgressAndNeverExpires)
{
  nmm::ManagerRecoveryTestAccess::latch(*manager);
  ASSERT_TRUE(nmm::ManagerRecoveryTestAccess::active(*manager));
  nmm::ManagerRecoveryTestAccess::check_latch(*manager);
}
TEST_F(ManagerRecoveryTest, CompleteUsesMeasuredExitNormalPlanningAndZeroOrdinaryCount)
{
  nmm::ManagerRecoveryTestAccess::latch(*manager);
  nmm::ManagerRecoveryTestAccess::complete(*manager);
}
TEST_F(ManagerRecoveryTest, RecoveryExitSurvivesRetryButIsConsumedBeforeNextTaskPlan)
{
  nmm::ManagerRecoveryTestAccess::latch(*manager);
  nmm::ManagerRecoveryTestAccess::complete(*manager);
  nmm::ManagerRecoveryTestAccess::handoff_lifecycle(*manager);
}
TEST_F(ManagerRecoveryTest, StrictPlanRejectionCannotRequestLegacyRetreat)
{
  nmm::ManagerRecoveryTestAccess::rejection_after_goal(*manager);
}
TEST_F(ManagerRecoveryTest, StrictHeadingFaultEndsInFaultRatherThanWaitingForRetreat)
{
  nmm::ManagerRecoveryTestAccess::heading_fault(*manager);
}
TEST_F(ManagerRecoveryTest, AbortAndResetRejectDurableStateAndLateServiceResponse)
{
  nmm::ManagerRecoveryTestAccess::latch(*manager);
  nmm::ManagerRecoveryTestAccess::stale_after_abort(*manager);
}
TEST_F(ManagerRecoveryTest, TerminalDurableMessageCannotStartRecovery)
{
  auto s = nmm::ManagerRecoveryTestAccess::state();
  s.phase = "complete";
  nmm::ManagerRecoveryTestAccess::message(*manager, s);
  EXPECT_FALSE(nmm::ManagerRecoveryTestAccess::active(*manager));
  s.phase = "failed";
  nmm::ManagerRecoveryTestAccess::message(*manager, s);
  EXPECT_FALSE(nmm::ManagerRecoveryTestAccess::active(*manager));
}
TEST_F(ManagerRecoveryTest, LocalPlanningFailureTriesNextCandidate)
{
  nmm::ManagerRecoveryTestAccess::latch(*manager);
  nmm::ManagerRecoveryTestAccess::selection_retry(*manager);
}
TEST_F(ManagerRecoveryTest, ExhaustedSpeechRetriesFaultAndCannotConfirmTask)
{
  nmm::ManagerRecoveryTestAccess::speech_failure(*manager);
}

TEST_F(ManagerRecoveryTest, FailureBeforeCandidatesEndsBrakingWait) {
  auto s=nmm::ManagerRecoveryTestAccess::state();
  nmm::ManagerRecoveryTestAccess::message(*manager,s);
  EXPECT_FALSE(nmm::ManagerRecoveryTestAccess::faulted(*manager));
  auto stale=s; stale.phase="failed"; stale.recovery_id++;
  nmm::ManagerRecoveryTestAccess::message(*manager,stale);
  EXPECT_FALSE(nmm::ManagerRecoveryTestAccess::faulted(*manager));
  s.phase="failed"; s.reason="FAULT_POSE_RECOVERY_FIXED_ENTRY_UNSAFE";
  nmm::ManagerRecoveryTestAccess::message(*manager,s);
  EXPECT_TRUE(nmm::ManagerRecoveryTestAccess::faulted(*manager));
}
