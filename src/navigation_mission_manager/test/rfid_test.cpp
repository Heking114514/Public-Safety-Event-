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
  static double settle(const MissionManager & m) {return m.rfid_stationary_settle_s_;}
  static void reset(MissionManager & m) {m.reset_rfid();}
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
