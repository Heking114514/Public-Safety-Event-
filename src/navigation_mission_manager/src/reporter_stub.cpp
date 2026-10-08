#include <memory>
#include <string>

#include <mission_control_interfaces/action/report_event.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace navigation_mission_manager
{

class ReporterStub : public rclcpp::Node
{
public:
  using ReportEvent = mission_control_interfaces::action::ReportEvent;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ReportEvent>;

  ReporterStub()
  : Node("event_reporter_stub")
  {
    const std::string action_name = declare_parameter(
      "action_name", std::string("/event_reporter/report"));
    server_ = rclcpp_action::create_server<ReportEvent>(
      this, action_name,
      [](const rclcpp_action::GoalUUID &, const std::shared_ptr<const ReportEvent::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<GoalHandle>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandle> handle) {execute(handle);});
  }

private:
  void execute(const std::shared_ptr<GoalHandle> & handle)
  {
    auto feedback = std::make_shared<ReportEvent::Feedback>();
    feedback->phase = ReportEvent::Feedback::PLAYING;
    feedback->status = "test reporter accepted event";
    handle->publish_feedback(feedback);
    auto result = std::make_shared<ReportEvent::Result>();
    result->success = true;
    result->report_id = std::to_string(now().nanoseconds());
    result->message = "test report completed";
    handle->succeed(result);
  }

  rclcpp_action::Server<ReportEvent>::SharedPtr server_;
};

}  // namespace navigation_mission_manager

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<navigation_mission_manager::ReporterStub>());
  rclcpp::shutdown();
  return 0;
}
