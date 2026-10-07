#include "visual_navigation/corner_shunt.hpp"

#include "visual_navigation/calibrated_turn.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>

namespace visual_navigation {
namespace {

constexpr double kPositionResolution = 0.0025;
constexpr double kYawResolution = 0.04363323129985824;
constexpr double kLinearResolution = 0.010;
constexpr double kAngularResolution = 0.06;
constexpr double kPi = 3.14159265358979323846;
constexpr double kSwitchCost = 0.60;
constexpr double kReverseCost = 0.03;
constexpr double kTightClearanceCost = 0.03;
constexpr std::size_t kMaximumStates = 100000;

struct SearchAction {
  CornerShuntTwist command;
};

struct SearchKey {
  int x{0};
  int y{0};
  int yaw{0};
  int linear{0};
  int angular{0};
  int action{-1};

  bool operator==(const SearchKey &other) const {
    return x == other.x && y == other.y && yaw == other.yaw &&
           linear == other.linear && angular == other.angular &&
           action == other.action;
  }
};

struct SearchKeyHash {
  std::size_t operator()(const SearchKey &key) const {
    std::size_t value = static_cast<std::size_t>(key.x + 4096);
    value = value * 8191U + static_cast<std::size_t>(key.y + 4096);
    value = value * 257U + static_cast<std::size_t>(key.yaw + 256);
    value = value * 257U + static_cast<std::size_t>(key.linear + 128);
    value = value * 257U + static_cast<std::size_t>(key.angular + 128);
    return value * 17U + static_cast<std::size_t>(key.action + 1);
  }
};

struct SearchNode {
  CornerShuntPose pose;
  CornerShuntTwist applied_twist;
  double cost{0.0};
  int parent{-1};
  int action{-1};
  SearchKey key;
};

struct QueueEntry {
  double priority{0.0};
  std::size_t node{0};

  bool operator<(const QueueEntry &other) const {
    return priority > other.priority;
  }
};

int Quantize(double value, double resolution) {
  return static_cast<int>(std::llround(value / resolution));
}

} // namespace

std::vector<CornerShuntTwist>
CornerShuntController::compute_plan(const CornerShuntContext &context) const {

  const double direction = turn_sign(context);
  const double forward_speed = std::max(0.005, config_.plan_linear_speed);
  const double reverse_speed = std::min(-0.005, config_.plan_reverse_speed);
  const double angular_speed = std::max(0.05, config_.plan_angular_speed);
  const std::vector<SearchAction> base_actions{
      {{forward_speed, angular_speed * direction}},
      {{forward_speed, 0.0}},
      {{forward_speed, -angular_speed * direction}},
      {{0.0, angular_speed * direction}},
      {{0.0, -angular_speed * direction}},
      {{reverse_speed, angular_speed * direction}},
      {{reverse_speed, 0.0}},
      {{reverse_speed, -angular_speed * direction}},
  };

  CornerShuntConfig clearance_config = config_;
  clearance_config.road_margin += std::max(0.0, config_.plan_clearance);
  CornerShuntController clearance_checker(clearance_config);

  const double control_frequency = std::max(1.0, config_.control_frequency);
  const int integration_samples = std::max(
      1, static_cast<int>(std::llround(std::max(0.01, config_.step_duration) *
                                       control_frequency)));
  const double control_period = 1.0 / control_frequency;
  const double duration = integration_samples * control_period;
  const double turn_amount = std::abs(turn_error(context));
  const double half_road = 0.5 * std::max(0.01, config_.road_width);
  const double margin =
      ClampShunt(config_.road_margin, 0.0, half_road - 1.0e-6);
  const double safe_half = half_road - margin;
  const double minimum_x =
      -std::min(std::max(0.0, config_.road_back) - margin, 0.18);
  const double maximum_x = safe_half + 0.03;
  const double minimum_y = -std::min(safe_half, 0.08);
  const double maximum_y =
      std::min(std::max(0.0, config_.road_forward) - margin,
               std::max(0.24, config_.target_y + 0.12));

  const auto make_key = [&](const CornerShuntPose &pose,
                            const CornerShuntTwist &applied, int action) {
    const auto local = local_turn_pose(context, pose);
    return SearchKey{Quantize(local.x, kPositionResolution),
                     Quantize(local.y, kPositionResolution),
                     Quantize(local.yaw, kYawResolution),
                     Quantize(applied.linear_x, kLinearResolution),
                     Quantize(applied.angular_z, kAngularResolution),
                     action};
  };
  const auto inside_search_window = [&](const CornerShuntPose &pose) {
    const auto local = local_turn_pose(context, pose);
    const bool yaw_inside =
        context.reverse_arrival
            ? std::abs(local.yaw) <= kPi + 0.28
            : local.yaw >= -0.25 && local.yaw <= turn_amount + 0.28;
    return local.x >= minimum_x && local.x <= maximum_x &&
           local.y >= minimum_y && local.y <= maximum_y && yaw_inside;
  };
  const double target_scale =
      ClampShunt(turn_amount / (0.5 * 3.14159265358979323846), 0.45, 1.40);
  const double target_x = config_.target_x * target_scale;
  const double target_y = config_.target_y * target_scale;
  const auto heuristic = [&](const CornerShuntPose &pose) {
    const auto local = local_turn_pose(context, pose);
    const double x_error = std::max(
        0.0, std::abs(local.x - target_x) -
                 std::max(0.0, config_.target_x_tolerance) * target_scale);
    const double y_error = std::max(
        0.0, target_y - std::max(0.0, config_.target_y_tolerance) - local.y);
    const double yaw_error = std::max(
        0.0, std::abs(NormalizeShuntAngle(context.target_heading - pose.yaw)) -
                 std::max(0.0, config_.target_yaw_tolerance));
    return 30.0 * std::hypot(x_error, y_error) + 5.0 * yaw_error;
  };
  const auto advance =
      [&](const CornerShuntPose &start, const CornerShuntTwist &start_twist,
          const CornerShuntTwist &command, CornerShuntPose *result,
          CornerShuntTwist *result_twist, bool *clearance_safe) {
        auto pose = start;
        auto applied = start_twist;
        bool has_clearance = true;
        for (int sample = 0; sample < integration_samples; ++sample) {
          applied = limited_command(applied, command, control_period);
          pose = integrate(pose, applied, control_period);
          if (!pose_is_safe(context, pose))
            return false;
          has_clearance =
              has_clearance && clearance_checker.pose_is_safe(context, pose);
        }
        *result = pose;
        *result_twist = applied;
        *clearance_safe = has_clearance;
        return true;
      };

  const auto local_start = local_turn_pose(context, context.pose);
  const auto &reference_start = kTightLeftTurnProfile.front();
  const bool calibrated_parameters =
      std::abs(forward_speed - kTightTurnLinearSpeed) <= 1.0e-9 &&
      std::abs(reverse_speed + kTightTurnLinearSpeed) <= 1.0e-9 &&
      std::abs(angular_speed - kTightTurnAngularSpeed) <= 1.0e-9 &&
      std::abs(duration - kTightTurnStepDuration) <= 1.0e-9 &&
      std::abs(control_frequency - kTightTurnControlFrequency) <= 1.0e-9 &&
      std::abs(config_.linear_acceleration - kTightTurnLinearAcceleration) <=
          1.0e-9 &&
      std::abs(config_.linear_deceleration - kTightTurnLinearDeceleration) <=
          1.0e-9 &&
      std::abs(config_.angular_acceleration - kTightTurnAngularAcceleration) <=
          1.0e-9 &&
      std::abs(config_.angular_deceleration - kTightTurnAngularDeceleration) <=
          1.0e-9;
  const bool calibrated_start =
      std::abs(turn_amount - 0.5 * 3.14159265358979323846) <= 0.05 &&
      std::abs(local_start.x - reference_start.axle_x) <= 0.008 &&
      std::abs(local_start.y) <= 0.008 && std::abs(local_start.yaw) <= 0.08 &&
      std::abs(context.applied_twist.linear_x) <= 0.01 &&
      std::abs(context.applied_twist.angular_z) <= 0.08;
  if (calibrated_parameters && calibrated_start) {
    const bool left_turn = direction > 0.0;
    const std::size_t action_count = left_turn ? kTightLeftTurnActions.size()
                                               : kTightRightTurnActions.size();
    std::vector<CornerShuntTwist> calibrated_plan;
    calibrated_plan.reserve(action_count);
    CornerShuntPose pose = context.pose;
    CornerShuntTwist applied = context.applied_twist;
    bool valid = true;
    for (std::size_t index = 0; index < action_count; ++index) {
      const uint8_t action = left_turn ? kTightLeftTurnActions[index]
                                       : kTightRightTurnActions[index];
      CornerShuntPose next_pose;
      CornerShuntTwist next_twist;
      bool clearance_safe = false;
      CornerShuntTwist command;
      switch (action) {
      case FORWARD_LEFT:
        command = {forward_speed, angular_speed};
        break;
      case FORWARD:
        command = {forward_speed, 0.0};
        break;
      case FORWARD_RIGHT:
        command = {forward_speed, -angular_speed};
        break;
      case PIVOT_LEFT:
        command = {0.0, angular_speed};
        break;
      case PIVOT_RIGHT:
        command = {0.0, -angular_speed};
        break;
      case REVERSE_LEFT:
        command = {reverse_speed, angular_speed};
        break;
      case REVERSE:
        command = {reverse_speed, 0.0};
        break;
      case REVERSE_RIGHT:
        command = {reverse_speed, -angular_speed};
        break;
      default:
        valid = false;
        continue;
      }
      if (!advance(pose, applied, command, &next_pose, &next_twist,
                   &clearance_safe) ||
          !inside_search_window(next_pose)) {
        valid = false;
        break;
      }
      calibrated_plan.push_back(command);
      pose = next_pose;
      applied = next_twist;
    }
    if (valid && target_pose_reached(context, pose)) {
      return calibrated_plan;
    }
  }

  const auto run_search = [&](const std::vector<SearchAction> &actions,
                              bool require_clearance) {
    std::vector<SearchNode> nodes;
    nodes.reserve(kMaximumStates);
    std::unordered_map<SearchKey, std::size_t, SearchKeyHash> best;
    best.reserve(kMaximumStates);
    std::priority_queue<QueueEntry> queue;

    const SearchKey start_key =
        make_key(context.pose, context.applied_twist, -1);
    nodes.push_back(
        {context.pose, context.applied_twist, 0.0, -1, -1, start_key});
    best[start_key] = 0;
    queue.push({2.0 * heuristic(context.pose), 0});

    int goal_node = -1;
    while (!queue.empty() && nodes.size() < kMaximumStates) {
      const QueueEntry entry = queue.top();
      queue.pop();
      const SearchNode current = nodes[entry.node];
      const auto best_entry = best.find(current.key);
      if (best_entry == best.end() || best_entry->second != entry.node)
        continue;
      if (target_pose_reached(context, current.pose)) {
        goal_node = static_cast<int>(entry.node);
        break;
      }

      for (std::size_t action_index = 0; action_index < actions.size();
           ++action_index) {
        CornerShuntPose next_pose;
        CornerShuntTwist next_twist;
        bool clearance_safe = false;
        const auto &command = actions[action_index].command;
        if (!advance(current.pose, current.applied_twist, command, &next_pose,
                     &next_twist, &clearance_safe) ||
            (require_clearance && !clearance_safe) ||
            !inside_search_window(next_pose))
          continue;

        const int next_action = static_cast<int>(action_index);
        const SearchKey key = make_key(next_pose, next_twist, next_action);
        double cost = current.cost + duration;
        if (current.action != next_action)
          cost += kSwitchCost;
        if (command.linear_x < 0.0)
          cost += kReverseCost;
        if (!clearance_safe)
          cost += kTightClearanceCost;

        const auto existing = best.find(key);
        if (existing != best.end() && nodes[existing->second].cost <= cost)
          continue;

        const std::size_t node_index = nodes.size();
        nodes.push_back({next_pose, next_twist, cost,
                         static_cast<int>(entry.node), next_action, key});
        best[key] = node_index;
        queue.push({cost + 2.0 * heuristic(next_pose), node_index});
      }
    }

    std::vector<CornerShuntTwist> reverse_plan;
    if (goal_node < 0)
      return reverse_plan;
    for (int node = goal_node; nodes[node].parent >= 0;
         node = nodes[node].parent) {
      reverse_plan.push_back(actions[nodes[node].action].command);
    }
    return std::vector<CornerShuntTwist>(reverse_plan.rbegin(),
                                         reverse_plan.rend());
  };

  if (config_.plan_clearance > 0.0 &&
      clearance_checker.pose_is_safe(context, context.pose)) {
    auto clearance_plan = run_search(base_actions, true);
    if (!clearance_plan.empty())
      return clearance_plan;
  }
  auto base_plan = run_search(base_actions, false);
  if (!base_plan.empty())
    return base_plan;

  auto fine_actions = base_actions;
  fine_actions.push_back({{forward_speed, 0.5 * angular_speed * direction}});
  fine_actions.push_back({{forward_speed, -0.5 * angular_speed * direction}});
  fine_actions.push_back({{reverse_speed, 0.5 * angular_speed * direction}});
  fine_actions.push_back({{reverse_speed, -0.5 * angular_speed * direction}});
  return run_search(fine_actions, false);
}

} // namespace visual_navigation
