#include <chrono>
#include <memory>
#include <vector>

#include <gtest/gtest.h>
#include "navigation_mission_manager/manager.hpp"

namespace navigation_mission_manager
{
struct ManagerRfidTestAccess
{
  static void prepare(MissionManager & m)
  {
    m.timer_->cancel();
    m.state_ = mission_control_interfaces::msg::MissionState::IDLE;
    m.hold_client_ = m.create_client<mission_control_interfaces::srv::SetMotionHold>(
      "/manager_rfid_test/hold");
  }
  static bool wait_for_hold(MissionManager & m)
  {return m.hold_client_->wait_for_service(std::chrono::seconds(2));}
  static void receive(MissionManager & m, int number)
  {
    auto message = std::make_shared<std_msgs::msg::Int32>();
    message->data = number;
    m.on_rfid_number(message);
  }
  static bool pending(const MissionManager & m) {return m.rfid_pending_card_.has_value();}
  static bool active(const MissionManager & m) {return m.rfid_active_.has_value();}
  static bool faulted(const MissionManager & m)
  {return m.state_ == mission_control_interfaces::msg::MissionState::FAULT;}
  static void route(MissionManager & m, bool acknowledged = true)
  {
    m.mission_->start({"1"});
    ActiveRoute route;
    route.route_id = 77U;
    route.acknowledged = acknowledged;
    route.spec = m.mission_->plan_spec();
    geometry_msgs::msg::PoseStamped pose;
    const auto xy = m.mission_->layout().task_positions.at("1");
    pose.pose.position.x = xy.first;
    pose.pose.position.y = xy.second;
    pose.pose.orientation.w = 1.0;
    route.plan.navigation_path.poses.push_back(pose);
    route.plan.waypoint_supernode_ids = {"s1"};
    m.active_route_ = route;
    m.current_waypoint_index_ = 0;
    m.pose_ = VehiclePose{xy.first, xy.second, 0.0, 0.0, steady_seconds()};
  }
  static void pause(MissionManager & m) {m.mission_->pause();}
  static void change_route(MissionManager & m) {++m.active_route_->route_id;}
  static void held(MissionManager & m)
  {
    std::lock_guard<std::mutex> lock(m.hold_state_mutex_);
    m.active_hold_sources_.insert("navigation_mission_manager/rfid");
  }
  static void service(MissionManager & m, double time, double speed = 0.0)
  {
    if (!m.pose_.has_value()) {
      m.pose_ = VehiclePose{};
    }
    m.pose_->received_at = steady_seconds();
    m.pose_->linear_speed = speed;
    m.service_rfid(time);
  }
  static void prepare_speech(MissionManager & m)
  {
    m.speaker_client_ = m.create_client<mission_control_interfaces::srv::SpeakText>(
      "/manager_rfid_test/speech");
    m.rfid_announcement_template_ = "已到达{number}号点";
  }
  static bool wait_for_speech(MissionManager & m)
  {return m.speaker_client_->wait_for_service(std::chrono::seconds(2));}
  static void announce(MissionManager & m)
  {
    m.rfid_phase_ = "announcing";
    m.service_rfid(steady_seconds() + 1.0);
  }
  static void finish(MissionManager & m) {m.finish_rfid_workflow();}
  static void move(MissionManager & m, double dx)
  {
    m.pose_->x += dx;
    m.pose_->received_at = steady_seconds();
  }
  static void stale_pose(MissionManager & m)
  {m.pose_->received_at = steady_seconds() - m.odom_timeout_s_ - 1.0;}
  static double radius(const MissionManager & m) {return m.rfid_detection_radius_m_;}
  static double stop_deadline(const MissionManager & m) {return m.rfid_next_hold_retry_at_;}
  static double settle(const MissionManager & m) {return m.rfid_stationary_settle_s_;}
  static void reset(MissionManager & m) {m.reset_rfid();}
  static void insight(MissionManager & m, const std::string & data)
  {
    auto message = std::make_shared<std_msgs::msg::String>();
    message->data = data;
    m.on_insight(message);
  }
  static std::vector<std::string> queued(const MissionManager & m)
  {
    std::vector<std::string> result;
    for (const auto & item : m.speech_queue_) {result.push_back(item.text);}
    return result;
  }
  static void enqueue(MissionManager & m, const std::string & text)
  {m.enqueue_speech_text(text, false);}
  static void tick_speech(MissionManager & m, double time)
  {m.service_speech_text(time);}
  static bool speech_pending(const MissionManager & m) {return m.speech_pending_;}
  static bool release_scheduled(const MissionManager & m)
  {return m.speech_release_at_.has_value();}
  static void held_speech(MissionManager & m)
  {
    m.active_hold_sources_.insert("navigation_mission_manager/speech");
  }
  static void held_text(MissionManager & m)
  {m.enqueue_speech_text("测试播报", true);}
};
}  // namespace navigation_mission_manager

namespace nmm = navigation_mission_manager;
using Hold = mission_control_interfaces::srv::SetMotionHold;

class ManagerRfidTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override
  {
    server = std::make_shared<rclcpp::Node>("manager_rfid_test_server");
    hold_service = server->create_service<Hold>(
      "/manager_rfid_test/hold",
      [this](Hold::Request::SharedPtr request, Hold::Response::SharedPtr response) {
        requests.push_back(*request);
        response->success = true;
        response->motion_held = true;
      });
    manager = std::make_shared<nmm::MissionManager>();
    nmm::ManagerRfidTestAccess::prepare(*manager);
    executor.add_node(server);
    executor.add_node(manager);
    ASSERT_TRUE(nmm::ManagerRfidTestAccess::wait_for_hold(*manager));
  }
  void collect(std::size_t count)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (requests.size() < count && std::chrono::steady_clock::now() < deadline) {
      executor.spin_once(std::chrono::milliseconds(10));
    }
    ASSERT_GE(requests.size(), count);
  }
  void receive(int number)
  {
    nmm::ManagerRfidTestAccess::receive(*manager, number);
    ASSERT_TRUE(nmm::ManagerRfidTestAccess::pending(*manager));
    nmm::ManagerRfidTestAccess::service(
      *manager, nmm::ManagerRfidTestAccess::stop_deadline(*manager));
    collect(1U);
    ASSERT_EQ(requests.front().source, "navigation_mission_manager/rfid");
    ASSERT_TRUE(requests.front().hold);
  }
  void stop_and_settle()
  {
    const double now = nmm::steady_seconds();
    nmm::ManagerRfidTestAccess::held(*manager);
    nmm::ManagerRfidTestAccess::service(*manager, now);
    EXPECT_TRUE(nmm::ManagerRfidTestAccess::pending(*manager));
    nmm::ManagerRfidTestAccess::service(
      *manager, now + nmm::ManagerRfidTestAccess::settle(*manager) + 0.01);
  }
  void expect_fault_hold()
  {
    EXPECT_TRUE(nmm::ManagerRfidTestAccess::faulted(*manager));
    EXPECT_FALSE(nmm::ManagerRfidTestAccess::pending(*manager));
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while ((requests.empty() || requests.back().source != "navigation_mission_manager/fault") &&
      std::chrono::steady_clock::now() < deadline)
    {
      executor.spin_once(std::chrono::milliseconds(10));
    }
    ASSERT_FALSE(requests.empty());
    for (const auto & request : requests) {
      EXPECT_TRUE(request.hold);
    }
    EXPECT_EQ(requests.back().source, "navigation_mission_manager/fault");
  }
  std::shared_ptr<nmm::MissionManager> manager;
  std::shared_ptr<rclcpp::Node> server;
  rclcpp::Service<Hold>::SharedPtr hold_service;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::vector<Hold::Request> requests;
};

TEST_F(ManagerRfidTest, StopsQuarterSecondAfterFirstCardWithoutExtendingOnDuplicates)
{
  const double before = nmm::steady_seconds();
  nmm::ManagerRfidTestAccess::receive(*manager, 12);
  const double deadline = nmm::ManagerRfidTestAccess::stop_deadline(*manager);
  // 生产 YAML 的读卡延时为 0.25 s；不应被重复读卡刷新。
  EXPECT_GE(deadline, before + 0.25);
  EXPECT_LE(deadline, nmm::steady_seconds() + 0.25);
  executor.spin_some();
  EXPECT_TRUE(requests.empty());
  nmm::ManagerRfidTestAccess::service(*manager, deadline - 0.001);
  executor.spin_some();
  EXPECT_TRUE(requests.empty());
  EXPECT_TRUE(nmm::ManagerRfidTestAccess::pending(*manager));
  EXPECT_FALSE(nmm::ManagerRfidTestAccess::faulted(*manager));
  nmm::ManagerRfidTestAccess::receive(*manager, 12);
  EXPECT_DOUBLE_EQ(nmm::ManagerRfidTestAccess::stop_deadline(*manager), deadline);
  nmm::ManagerRfidTestAccess::service(*manager, deadline);
  collect(1U);
  EXPECT_EQ(requests.front().source, "navigation_mission_manager/rfid");
  EXPECT_TRUE(requests.front().hold);
  EXPECT_TRUE(nmm::ManagerRfidTestAccess::pending(*manager));
}

TEST_F(ManagerRfidTest, ResetCancelsDelayedStop)
{
  nmm::ManagerRfidTestAccess::receive(*manager, 12);
  const double deadline = nmm::ManagerRfidTestAccess::stop_deadline(*manager);
  nmm::ManagerRfidTestAccess::reset(*manager);
  collect(1U);
  EXPECT_FALSE(requests.front().hold);
  nmm::ManagerRfidTestAccess::service(*manager, deadline + 1.0);
  executor.spin_some();
  EXPECT_EQ(requests.size(), 1U);
  EXPECT_FALSE(nmm::ManagerRfidTestAccess::pending(*manager));
}

TEST_F(ManagerRfidTest, IdleManagerStopsBeforeCheckingMissionAndKeepsVehicleStopped)
{
  receive(12);
  EXPECT_FALSE(nmm::ManagerRfidTestAccess::faulted(*manager));
  nmm::ManagerRfidTestAccess::service(*manager, nmm::steady_seconds());
  EXPECT_TRUE(nmm::ManagerRfidTestAccess::pending(*manager));
  EXPECT_FALSE(nmm::ManagerRfidTestAccess::faulted(*manager));
  stop_and_settle();
  expect_fault_hold();
  const auto request_count = requests.size();
  nmm::ManagerRfidTestAccess::reset(*manager);
  collect(request_count + 1U);
  EXPECT_EQ(requests.back().source, "navigation_mission_manager/rfid");
  EXPECT_FALSE(requests.back().hold);
}

TEST_F(ManagerRfidTest, PausedMissionStillRequestsStopBeforeValidation)
{
  nmm::ManagerRfidTestAccess::route(*manager);
  nmm::ManagerRfidTestAccess::pause(*manager);
  receive(12);
  stop_and_settle();
  expect_fault_hold();
}

TEST_F(ManagerRfidTest, UnacknowledgedRouteStillRequestsStopBeforeValidation)
{
  nmm::ManagerRfidTestAccess::route(*manager, false);
  receive(12);
  stop_and_settle();
  expect_fault_hold();
}

TEST_F(ManagerRfidTest, ChangedRouteCannotReleaseStopForOldCard)
{
  nmm::ManagerRfidTestAccess::route(*manager);
  receive(12);
  nmm::ManagerRfidTestAccess::change_route(*manager);
  stop_and_settle();
  expect_fault_hold();
}

TEST_F(ManagerRfidTest, ValidCardNeedsHoldConfirmationAndStableStationaryPose)
{
  nmm::ManagerRfidTestAccess::route(*manager);
  receive(12);
  double now = nmm::steady_seconds();
  nmm::ManagerRfidTestAccess::service(*manager, now);
  EXPECT_FALSE(nmm::ManagerRfidTestAccess::active(*manager));
  nmm::ManagerRfidTestAccess::held(*manager);
  nmm::ManagerRfidTestAccess::service(*manager, now, 0.1);
  EXPECT_FALSE(nmm::ManagerRfidTestAccess::active(*manager));
  nmm::ManagerRfidTestAccess::service(*manager, now);
  EXPECT_FALSE(nmm::ManagerRfidTestAccess::active(*manager));
  nmm::ManagerRfidTestAccess::service(
    *manager, now + nmm::ManagerRfidTestAccess::settle(*manager) + 0.01);
  EXPECT_TRUE(nmm::ManagerRfidTestAccess::active(*manager));
  EXPECT_FALSE(nmm::ManagerRfidTestAccess::pending(*manager));
  EXPECT_FALSE(nmm::ManagerRfidTestAccess::faulted(*manager));
}

TEST_F(ManagerRfidTest, CompletedCardNearCheckpointDoesNotStopAgainAfterRouteChange)
{
  nmm::ManagerRfidTestAccess::route(*manager);
  receive(12);
  stop_and_settle();
  ASSERT_TRUE(nmm::ManagerRfidTestAccess::active(*manager));
  nmm::ManagerRfidTestAccess::finish(*manager);
  collect(2U);
  nmm::ManagerRfidTestAccess::change_route(*manager);
  nmm::ManagerRfidTestAccess::move(*manager, 0.044);
  for (int i = 0; i < 3; ++i) {
    nmm::ManagerRfidTestAccess::receive(*manager, 12);
    EXPECT_FALSE(nmm::ManagerRfidTestAccess::pending(*manager));
    nmm::ManagerRfidTestAccess::service(*manager, nmm::steady_seconds() + 10.0);
  }
  executor.spin_some();
  EXPECT_EQ(requests.size(), 2U);
  nmm::ManagerRfidTestAccess::receive(*manager, 11);
  EXPECT_TRUE(nmm::ManagerRfidTestAccess::pending(*manager));
  nmm::ManagerRfidTestAccess::service(
    *manager, nmm::ManagerRfidTestAccess::stop_deadline(*manager));
  collect(3U);
  EXPECT_TRUE(requests.back().hold);
}

TEST_F(ManagerRfidTest, LeavingCheckpointRearmsSameCardEvenWhenReturning)
{
  nmm::ManagerRfidTestAccess::route(*manager);
  receive(12);
  stop_and_settle();
  ASSERT_TRUE(nmm::ManagerRfidTestAccess::active(*manager));
  nmm::ManagerRfidTestAccess::finish(*manager);
  collect(2U);
  const double departure = nmm::ManagerRfidTestAccess::radius(*manager) + 0.01;
  nmm::ManagerRfidTestAccess::move(*manager, departure);
  nmm::ManagerRfidTestAccess::service(*manager, nmm::steady_seconds());
  nmm::ManagerRfidTestAccess::move(*manager, -departure);
  nmm::ManagerRfidTestAccess::receive(*manager, 12);
  EXPECT_TRUE(nmm::ManagerRfidTestAccess::pending(*manager));
  nmm::ManagerRfidTestAccess::service(
    *manager, nmm::ManagerRfidTestAccess::stop_deadline(*manager));
  collect(3U);
  EXPECT_TRUE(requests.back().hold);
}

TEST_F(ManagerRfidTest, StalePoseDoesNotSuppressCardAfterCompletion)
{
  nmm::ManagerRfidTestAccess::route(*manager);
  receive(12);
  stop_and_settle();
  ASSERT_TRUE(nmm::ManagerRfidTestAccess::active(*manager));
  nmm::ManagerRfidTestAccess::finish(*manager);
  collect(2U);
  nmm::ManagerRfidTestAccess::stale_pose(*manager);
  nmm::ManagerRfidTestAccess::receive(*manager, 12);
  EXPECT_TRUE(nmm::ManagerRfidTestAccess::pending(*manager));
}

TEST_F(ManagerRfidTest, ResetClearsCompletedCardSuppression)
{
  nmm::ManagerRfidTestAccess::route(*manager);
  receive(12);
  stop_and_settle();
  ASSERT_TRUE(nmm::ManagerRfidTestAccess::active(*manager));
  nmm::ManagerRfidTestAccess::finish(*manager);
  collect(2U);
  nmm::ManagerRfidTestAccess::reset(*manager);
  nmm::ManagerRfidTestAccess::receive(*manager, 12);
  EXPECT_TRUE(nmm::ManagerRfidTestAccess::pending(*manager));
}

TEST_F(ManagerRfidTest, SpeechUsesObservedCardWhenNumberChangesAtSamePhysicalCheckpoint)
{
  using Speech = mission_control_interfaces::srv::SpeakText;
  std::vector<std::string> spoken;
  auto speech_service = server->create_service<Speech>(
    "/manager_rfid_test/speech",
    [&spoken](Speech::Request::SharedPtr request, Speech::Response::SharedPtr response) {
      spoken.push_back(request->text);
      response->success = true;
    });
  nmm::ManagerRfidTestAccess::prepare_speech(*manager);
  ASSERT_TRUE(nmm::ManagerRfidTestAccess::wait_for_speech(*manager));
  for (const int card : {12, 11}) {
    nmm::ManagerRfidTestAccess::reset(*manager);
    if (card == 11) {
      collect(3U);
      executor.spin_some();
    }
    nmm::ManagerRfidTestAccess::route(*manager);
    requests.clear();
    receive(card);
    stop_and_settle();
    ASSERT_TRUE(nmm::ManagerRfidTestAccess::active(*manager));
    nmm::ManagerRfidTestAccess::announce(*manager);
    const std::size_t expected_count = card == 12 ? 1U : 2U;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (spoken.size() < expected_count && std::chrono::steady_clock::now() < deadline) {
      executor.spin_once(std::chrono::milliseconds(10));
    }
    ASSERT_EQ(spoken.size(), expected_count);
    EXPECT_EQ(spoken.back(), card == 12 ? "已到达十二号点" : "已到达十一号点");
    // Complete the visit before resetting the next visit at the same point.
    executor.spin_some();
    nmm::ManagerRfidTestAccess::finish(*manager);
    collect(2U);
    executor.spin_some();
  }
}

TEST_F(ManagerRfidTest, SpeechRejectsInvalidBytesAndAccepts128ByteBoundary)
{
  using Access = nmm::ManagerRfidTestAccess;
  Access::enqueue(*manager, std::string(128U, 'a'));
  Access::enqueue(*manager, std::string(129U, 'a'));
  Access::enqueue(*manager, "   ");
  Access::enqueue(*manager, "a\r\nb");
  Access::enqueue(*manager, std::string("a\0b", 3U));
  ASSERT_EQ(Access::queued(*manager).size(), 1U);
}

TEST_F(ManagerRfidTest, SpeechHoldWaitsForServiceCompletionBeforeReleaseTimer)
{
  using Speech = mission_control_interfaces::srv::SpeakText;
  using Access = nmm::ManagerRfidTestAccess;
  auto speech_service = server->create_service<Speech>(
    "/manager_rfid_test/speech",
    [](Speech::Request::SharedPtr, Speech::Response::SharedPtr response) {
      response->success = true;
    });
  Access::prepare_speech(*manager);
  ASSERT_TRUE(Access::wait_for_speech(*manager));
  Access::held_text(*manager);
  Access::held_speech(*manager);
  Access::tick_speech(*manager, nmm::steady_seconds());
  // Leave the server unspun to keep its completion response pending.
  Access::tick_speech(*manager, nmm::steady_seconds() + 1000.0);
  EXPECT_TRUE(Access::speech_pending(*manager));
  EXPECT_FALSE(Access::release_scheduled(*manager));
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (Access::speech_pending(*manager) && std::chrono::steady_clock::now() < deadline) {
    executor.spin_once(std::chrono::milliseconds(10));
  }
  EXPECT_FALSE(Access::speech_pending(*manager));
  EXPECT_TRUE(Access::release_scheduled(*manager));
}

TEST_F(ManagerRfidTest, ResetIgnoresLateSpeechCompletion)
{
  using Speech = mission_control_interfaces::srv::SpeakText;
  using Access = nmm::ManagerRfidTestAccess;
  auto speech_service = server->create_service<Speech>(
    "/manager_rfid_test/speech",
    [](Speech::Request::SharedPtr, Speech::Response::SharedPtr response) {
      response->success = true;
    });
  Access::prepare_speech(*manager);
  ASSERT_TRUE(Access::wait_for_speech(*manager));
  Access::held_text(*manager);
  Access::held_speech(*manager);
  Access::tick_speech(*manager, nmm::steady_seconds());
  ASSERT_TRUE(Access::speech_pending(*manager));
  Access::reset(*manager);
  collect(1U);
  executor.spin_some();
  EXPECT_FALSE(Access::speech_pending(*manager));
  EXPECT_FALSE(Access::release_scheduled(*manager));
}

TEST_F(ManagerRfidTest, VisualSpeechWaitsWhileRfidWorkflowIsActive)
{
  using Speech = mission_control_interfaces::srv::SpeakText;
  using Access = nmm::ManagerRfidTestAccess;
  std::vector<std::string> spoken;
  auto speech_service = server->create_service<Speech>(
    "/manager_rfid_test/speech",
    [&spoken](Speech::Request::SharedPtr request, Speech::Response::SharedPtr response) {
      spoken.push_back(request->text);
      response->success = true;
    });
  Access::prepare_speech(*manager);
  ASSERT_TRUE(Access::wait_for_speech(*manager));
  Access::route(*manager);
  receive(12);
  stop_and_settle();
  Access::insight(*manager, R"({"targets":["f2"]})");
  Access::tick_speech(*manager, nmm::steady_seconds());
  executor.spin_some();
  EXPECT_FALSE(Access::speech_pending(*manager));
  EXPECT_TRUE(spoken.empty());
  Access::finish(*manager);
  Access::tick_speech(*manager, nmm::steady_seconds());
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (spoken.empty() && std::chrono::steady_clock::now() < deadline) {
    executor.spin_once(std::chrono::milliseconds(10));
  }
  ASSERT_EQ(spoken.size(), 1U);
  EXPECT_EQ(spoken.front(), "发现2号嫌疑人");
}

TEST_F(ManagerRfidTest, InvalidCardIsRejectedOnlyAfterStop)
{
  nmm::ManagerRfidTestAccess::route(*manager);
  receive(13);
  stop_and_settle();
  EXPECT_FALSE(nmm::ManagerRfidTestAccess::pending(*manager));
  EXPECT_FALSE(nmm::ManagerRfidTestAccess::active(*manager));
  collect(2U);
  EXPECT_EQ(requests.back().source, "navigation_mission_manager/rfid");
  EXPECT_FALSE(requests.back().hold);
}
