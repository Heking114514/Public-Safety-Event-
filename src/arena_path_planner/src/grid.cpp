#include "arena_path_planner/planner_internal.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <tuple>

namespace arena_path_planner
{
ArenaPlanner::ArenaPlanner(PlannerConfig config, bool snap_inspection_nodes)
: config_(std::move(config))
{
  ValidatePlannerConfig(config_);
  if (config_.vehicle_length > 0.0) {
    const double translational_radius = 0.5 * std::max(
      config_.vehicle_length, config_.vehicle_width) + config_.safety_margin;
    const double rotation_radius = 0.5 * std::hypot(
      config_.vehicle_length, config_.vehicle_width) + config_.safety_margin;
    // The grid is a fast centre-path search.  It only needs the largest body
    // half-axis for translation; every emitted segment is then checked against
    // the full oriented rectangle below.  Using the rotation circle here would
    // wrongly reject a 30 cm straight lane that this chassis can traverse.
    config_.inflation_radius = std::max(config_.inflation_radius, translational_radius);
    config_.preferred_clearance = std::max(
      config_.preferred_clearance, translational_radius + config_.tracking_margin);
    config_.minimum_turning_radius = std::max(
      config_.minimum_turning_radius, rotation_radius);
  }
  BuildGrid();
  if (snap_inspection_nodes) {
    SnapInspectionNodesToFreeSpace();
  }
}

int ArenaPlanner::Index(int column, int row) const
{
  return row * columns_ + column;
}

bool ArenaPlanner::InBounds(int column, int row) const
{
  return column >= 0 && column < columns_ && row >= 0 && row < rows_;
}

bool ArenaPlanner::CellIsFree(int column, int row) const
{
  return InBounds(column, row) && !occupied_[Index(column, row)];
}

bool ArenaPlanner::BaseSpaceIsFree(const Point & point) const
{
  if (!IsFinite(point)) {
    return false;
  }
  const bool in_staging_region = std::any_of(
    config_.staging_regions.begin(), config_.staging_regions.end(),
    [&point](const Rectangle & rectangle) {return Contains(rectangle, point);});
  if (in_staging_region) {
    return true;
  }
  if (point.x < 0.0 || point.y < 0.0 || point.x >= config_.width ||
    point.y >= config_.height)
  {
    return false;
  }
  const Cell cell = WorldToCell(point);
  return !base_occupied_[Index(cell.first, cell.second)];
}

ArenaPlanner::Cell ArenaPlanner::WorldToCell(const Point & point) const
{
  if (!IsFinite(point)) {
    throw std::invalid_argument("grid point must be finite");
  }
  const double column = std::floor(point.x / config_.resolution);
  const double row = std::floor(point.y / config_.resolution);
  if (column < static_cast<double>(std::numeric_limits<int>::min()) ||
    column > static_cast<double>(std::numeric_limits<int>::max()) ||
    row < static_cast<double>(std::numeric_limits<int>::min()) ||
    row > static_cast<double>(std::numeric_limits<int>::max()))
  {
    throw std::out_of_range("grid point is outside integer coordinate range");
  }
  return {
    std::clamp(static_cast<int>(column), 0, columns_ - 1),
    std::clamp(static_cast<int>(row), 0, rows_ - 1)};
}

Point ArenaPlanner::CellToWorld(const Cell & cell) const
{
  return {
    (static_cast<double>(cell.first) + 0.5) * config_.resolution,
    (static_cast<double>(cell.second) + 0.5) * config_.resolution};
}

void ArenaPlanner::BuildGrid()
{
  columns_ = static_cast<int>(std::ceil(config_.width / config_.resolution));
  rows_ = static_cast<int>(std::ceil(config_.height / config_.resolution));
  base_occupied_.assign(columns_ * rows_, true);
  for (int row = 0; row < rows_; ++row) {
    for (int column = 0; column < columns_; ++column) {
      const Point point = CellToWorld({column, row});
      const bool inside = std::any_of(
        config_.free_regions.begin(), config_.free_regions.end(),
        [&point](const Rectangle & rectangle) {return Contains(rectangle, point);});
      const bool obstacle = std::any_of(
        config_.obstacles.begin(), config_.obstacles.end(),
        [&point](const Rectangle & rectangle) {return Contains(rectangle, point);});
      base_occupied_[Index(column, row)] = !inside || obstacle;
    }
  }

  occupied_ = base_occupied_;
  // A* always follows cell centres. The full rectangular footprint is checked
  // at centimetre resolution for every emitted segment, so adding a cell
  // diagonal here would double-count the safety margin and close valid narrow
  // lanes.
  const double inflation_distance = config_.inflation_radius + 0.5 * config_.resolution;
  const int inflation_cells = static_cast<int>(
    std::ceil(inflation_distance / config_.resolution));
  for (int row = 0; row < rows_; ++row) {
    for (int column = 0; column < columns_; ++column) {
      if (base_occupied_[Index(column, row)]) {
        continue;
      }
      for (int row_offset = -inflation_cells; row_offset <= inflation_cells; ++row_offset) {
        for (int column_offset = -inflation_cells;
          column_offset <= inflation_cells; ++column_offset)
        {
          if (std::hypot(column_offset, row_offset) * config_.resolution >
            inflation_distance + 1.0e-9)
          {
            continue;
          }
          const int test_column = column + column_offset;
          const int test_row = row + row_offset;
          const Point test_point = CellToWorld({test_column, test_row});
          if (!BaseSpaceIsFree(test_point))
          {
            occupied_[Index(column, row)] = true;
          }
        }
      }
    }
  }

  clearance_.assign(columns_ * rows_, std::numeric_limits<double>::infinity());
  using Entry = std::tuple<double, int, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> frontier;
  for (int row = 0; row < rows_; ++row) {
    for (int column = 0; column < columns_; ++column) {
      if (base_occupied_[Index(column, row)]) {
        clearance_[Index(column, row)] = 0.0;
        frontier.emplace(0.0, column, row);
      }
    }
  }
  const std::vector<std::tuple<int, int, double>> steps{
    {1, 0, 1.0}, {-1, 0, 1.0}, {0, 1, 1.0}, {0, -1, 1.0},
    {1, 1, std::sqrt(2.0)}, {1, -1, std::sqrt(2.0)},
    {-1, 1, std::sqrt(2.0)}, {-1, -1, std::sqrt(2.0)}};
  while (!frontier.empty()) {
    const auto [distance, column, row] = frontier.top();
    frontier.pop();
    if (distance > clearance_[Index(column, row)] + 1.0e-12) {
      continue;
    }
    for (const auto & [column_delta, row_delta, multiplier] : steps) {
      const int next_column = column + column_delta;
      const int next_row = row + row_delta;
      if (!InBounds(next_column, next_row)) {
        continue;
      }
      const double candidate = distance + multiplier * config_.resolution;
      if (candidate + 1.0e-12 < clearance_[Index(next_column, next_row)]) {
        clearance_[Index(next_column, next_row)] = candidate;
        frontier.emplace(candidate, next_column, next_row);
      }
    }
  }
}

void ArenaPlanner::SnapInspectionNodesToFreeSpace()
{
  // The inspection graph was exported from an older coarse grid. Some of its
  // node centres lie a few centimetres from a wall, which was acceptable for
  // a point robot but not for the chassis centre. Keep graph connectivity and
  // move only those stale centres to their nearest valid centre-path cell.
  for (Point & node : config_.inspection_nodes) {
    if (IsFree(node)) {
      continue;
    }
    double best_distance = std::numeric_limits<double>::infinity();
    Cell best_cell{-1, -1};
    for (int row = 0; row < rows_; ++row) {
      for (int column = 0; column < columns_; ++column) {
        if (!CellIsFree(column, row)) {
          continue;
        }
        const Point candidate = CellToWorld({column, row});
        const double distance = Distance(node, candidate);
        if (distance + 1.0e-12 < best_distance) {
          best_distance = distance;
          best_cell = {column, row};
        }
      }
    }
    if (best_cell.first >= 0) {
      node = CellToWorld(best_cell);
    }
  }
}

bool ArenaPlanner::IsFree(const Point & point) const
{
  if (!IsFinite(point) || point.x < 0.0 || point.y < 0.0 || point.x >= config_.width ||
    point.y >= config_.height)
  {
    return false;
  }
  const Cell cell = WorldToCell(point);
  return CellIsFree(cell.first, cell.second);
}

bool ArenaPlanner::PoseIsFree(const Point & point, double yaw) const
{
  if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(yaw)) {
    return false;
  }
  if (config_.vehicle_length <= 0.0) {
    return BaseSpaceIsFree(point);
  }
  const double half_length = 0.5 * config_.vehicle_length + config_.safety_margin;
  const double half_width = 0.5 * config_.vehicle_width + config_.safety_margin;
  const double spacing = std::max(0.005, config_.resolution * 0.25);
  const int length_samples = std::max(
    1, static_cast<int>(std::ceil(2.0 * half_length / spacing)));
  const int width_samples = std::max(
    1, static_cast<int>(std::ceil(2.0 * half_width / spacing)));
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  for (int length_index = 0; length_index <= length_samples; ++length_index) {
    const double local_x = -half_length +
      2.0 * half_length * static_cast<double>(length_index) / length_samples;
    for (int width_index = 0; width_index <= width_samples; ++width_index) {
      const double local_y = -half_width +
        2.0 * half_width * static_cast<double>(width_index) / width_samples;
      if (!BaseSpaceIsFree({
            point.x + cosine * local_x - sine * local_y,
            point.y + sine * local_x + cosine * local_y}))
      {
        return false;
      }
    }
  }
  return true;
}

bool ArenaPlanner::RotationIsFree(
  const Point & point, double from_yaw, double to_yaw) const
{
  if (!std::isfinite(from_yaw) || !std::isfinite(to_yaw)) {
    return false;
  }
  const double delta = NormalizeAngle(to_yaw - from_yaw);
  const int samples = std::max(
    1, static_cast<int>(std::ceil(std::abs(delta) / config_.maximum_heading_step)));
  for (int index = 0; index <= samples; ++index) {
    if (!PoseIsFree(point, from_yaw + delta * static_cast<double>(index) / samples)) {
      return false;
    }
  }
  return true;
}

bool ArenaPlanner::IsTurnJunction(const Point & point, double tolerance) const
{
  const auto near_point = [&point, tolerance](const Point & junction) {
      return Distance(point, junction) <= tolerance;
    };
  return std::any_of(config_.inspection_nodes.begin(), config_.inspection_nodes.end(), near_point) ||
         std::any_of(config_.turn_junctions.begin(), config_.turn_junctions.end(), near_point);
}

bool ArenaPlanner::RouteTurnsAreAllowed(const std::vector<Point> & points) const
{
  if (!config_.turns_at_junctions_only) {
    return true;
  }
  constexpr double kNearReversalThreshold = 2.6;
  constexpr double kMajorTurnThreshold = 1.0;
  const double consecutive_turn_distance =
    std::max(0.35, config_.minimum_turning_radius + 2.0 * config_.resolution);
  std::size_t previous_major_turn = points.size();
  for (std::size_t index = 1; index + 1 < points.size(); ++index) {
    if (Distance(points[index - 1], points[index]) <= 1.0e-6 ||
      Distance(points[index], points[index + 1]) <= 1.0e-6)
    {
      continue;
    }
    const double incoming = std::atan2(
      points[index].y - points[index - 1].y,
      points[index].x - points[index - 1].x);
    const double outgoing = std::atan2(
      points[index + 1].y - points[index].y,
      points[index + 1].x - points[index].x);
    const double heading_change = std::abs(NormalizeAngle(outgoing - incoming));
    if (heading_change < config_.in_place_turn_heading_threshold) {
      continue;
    }
    if (heading_change >= kNearReversalThreshold) {
      return false;
    }
    if (!IsTurnJunction(points[index], kTurnJunctionOperatingTolerance))
    {
      return false;
    }
    if (heading_change >= kMajorTurnThreshold) {
      if (previous_major_turn + 1 == index &&
        Distance(points[previous_major_turn], points[index]) <= consecutive_turn_distance)
      {
        return false;
      }
      previous_major_turn = index;
    }
  }
  return true;
}

double ArenaPlanner::Clearance(const Point & point) const
{
  if (!IsFree(point)) {
    return 0.0;
  }
  const Cell cell = WorldToCell(point);
  const Point centre = CellToWorld(cell);
  const double boundary = std::min(
    {centre.x, centre.y, config_.width - centre.x, config_.height - centre.y});
  return std::max(0.0, std::min(clearance_[Index(cell.first, cell.second)], boundary));
}

bool ArenaPlanner::SegmentIsFree(const Point & start, const Point & end) const
{
  if (!IsFinite(start) || !IsFinite(end)) {
    return false;
  }
  const double distance = Distance(start, end);
  const double spacing = std::max(0.005, config_.resolution * 0.2);
  if (!std::isfinite(distance) ||
    distance / spacing > static_cast<double>(std::numeric_limits<int>::max() - 1))
  {
    return false;
  }
  const int count = std::max(1, static_cast<int>(std::ceil(distance / spacing)));
  const double yaw = std::atan2(end.y - start.y, end.x - start.x);
  for (int index = 0; index <= count; ++index) {
    const double ratio = static_cast<double>(index) / count;
    if (!PoseIsFree(
        {start.x + ratio * (end.x - start.x),
          start.y + ratio * (end.y - start.y)}, yaw))
    {
      return false;
    }
  }
  return true;
}

bool ArenaPlanner::ProjectToNearestFreePose(
  const Pose & pose, double maximum_distance, Pose & projected) const
{
  if (!IsFinite(pose) || !std::isfinite(maximum_distance) || maximum_distance < 0.0) {
    return false;
  }
  if (IsFree(pose.position) && PoseIsFree(pose.position, pose.yaw)) {
    projected = pose;
    return true;
  }

  double best_distance = maximum_distance + 1.0e-12;
  Cell best_cell{-1, -1};
  for (int row = 0; row < rows_; ++row) {
    for (int column = 0; column < columns_; ++column) {
      if (!CellIsFree(column, row)) {
        continue;
      }
      const Point candidate = CellToWorld({column, row});
      const double distance = Distance(pose.position, candidate);
      if (distance > best_distance || !PoseIsFree(candidate, pose.yaw)) {
        continue;
      }
      if (distance + 1.0e-12 < best_distance || best_cell.first < 0 ||
        Clearance(candidate) > Clearance(CellToWorld(best_cell)) + 1.0e-12)
      {
        best_distance = distance;
        best_cell = {column, row};
      }
    }
  }
  if (best_cell.first < 0) {
    return false;
  }
  projected = {CellToWorld(best_cell), pose.yaw};
  return true;
}

std::vector<int8_t> ArenaPlanner::OccupancyData() const
{
  std::vector<int8_t> result;
  result.reserve(occupied_.size());
  for (bool value : occupied_) {
    result.push_back(value ? 100 : 0);
  }
  return result;
}

ArenaPlanner::GridPath ArenaPlanner::AStar(
  const Point & start, const Point & goal, double initial_heading) const
{
  if (!IsFree(start) || !IsFree(goal)) {
    throw std::runtime_error("start or target is outside collision-free space");
  }
  const Cell start_cell = WorldToCell(start);
  const Cell goal_cell = WorldToCell(goal);
  if (!CellIsFree(start_cell.first, start_cell.second) ||
    !CellIsFree(goal_cell.first, goal_cell.second))
  {
    throw std::runtime_error("start or target is occupied");
  }
  // Direction is part of the search state. This lets the planner optimize the
  // quantity that matters to the differential-drive chassis: first pivot
  // count, then clearance-weighted travel distance. Four-connected motion also
  // guarantees that ordinary connector legs are horizontal or vertical.
  constexpr int kDirectionCount = 4;
  constexpr int kNoDirection = 4;
  const int column_steps[kDirectionCount] = {1, 0, -1, 0};
  const int row_steps[kDirectionCount] = {0, 1, 0, -1};
  const double direction_yaws[kDirectionCount] = {
    0.0, 0.5 * std::acos(-1.0), std::acos(-1.0), -0.5 * std::acos(-1.0)};
  using Entry = std::tuple<double, int, double, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> frontier;
  const int start_index = Index(start_cell.first, start_cell.second);
  const int goal_index = Index(goal_cell.first, goal_cell.second);
  const bool heading_constrained = std::isfinite(initial_heading);
  if (start_index == goal_index) {
    return {{start_cell}, 0, 0.0};
  }
  const std::uint64_t path_cache_key =
    (static_cast<std::uint64_t>(static_cast<std::uint32_t>(start_index)) << 32U) |
    static_cast<std::uint32_t>(goal_index);
  const auto cached_path = grid_path_cache_.find(path_cache_key);
  if (!heading_constrained && cached_path != grid_path_cache_.end()) {
    return cached_path->second;
  }
  const auto cache_path = [this, path_cache_key, start_index, goal_index,
      heading_constrained](
      GridPath result) {
      if (heading_constrained) {
        return result;
      }
      grid_path_cache_.emplace(path_cache_key, result);
      const std::uint64_t reverse_cache_key =
        (static_cast<std::uint64_t>(static_cast<std::uint32_t>(goal_index)) << 32U) |
        static_cast<std::uint32_t>(start_index);
      GridPath reversed = result;
      std::reverse(reversed.cells.begin(), reversed.cells.end());
      grid_path_cache_.emplace(reverse_cache_key, std::move(reversed));
      return result;
    };
  constexpr double kAxisTolerance = 1.0e-9;
  constexpr double kNearReversalThreshold = 2.6;
  const auto initial_turn_count = [initial_heading](double departure) {
      if (!std::isfinite(initial_heading)) {
        return 0;
      }
      const double change = std::abs(NormalizeAngle(departure - initial_heading));
      if (change <= 1.0e-3) {
        return 0;
      }
      return change > 0.75 * std::acos(-1.0) ? 2 : 1;
    };
  const auto departure_is_allowed = [this, &start, initial_heading,
      heading_constrained](double departure) {
      if (!heading_constrained) {
        return true;
      }
      const double change = std::abs(NormalizeAngle(departure - initial_heading));
      if (change <= 1.0e-3) {
        return true;
      }
      if (change >= kNearReversalThreshold) {
        return false;
      }
      if (config_.turns_at_junctions_only &&
        change >= config_.in_place_turn_heading_threshold &&
        !IsTurnJunction(start, kTurnJunctionOperatingTolerance))
      {
        return false;
      }
      const bool must_fit_rotation =
        change > kNearReversalThreshold || !config_.allow_in_place_turns;
      return !must_fit_rotation ||
             (config_.turns_at_junctions_only &&
             IsTurnJunction(start, kTurnJunctionOperatingTolerance)) ||
             RotationIsFree(start, initial_heading, departure);
    };
  if ((std::abs(start.x - goal.x) <= kAxisTolerance ||
    std::abs(start.y - goal.y) <= kAxisTolerance) &&
    SegmentIsFree(start, goal))
  {
    const double departure = std::atan2(goal.y - start.y, goal.x - start.x);
    if (departure_is_allowed(departure)) {
      return cache_path({
        {start_cell, goal_cell}, initial_turn_count(departure),
        Distance(start, goal), heading_constrained});
    }
  }
  const int directional_state_count = columns_ * rows_ * kDirectionCount;
  const int start_state = directional_state_count;
  std::vector<int> turns(
    static_cast<std::size_t>(directional_state_count + 1),
    std::numeric_limits<int>::max());
  std::vector<double> costs(
    static_cast<std::size_t>(directional_state_count + 1),
    std::numeric_limits<double>::infinity());
  std::vector<int> parents(
    static_cast<std::size_t>(directional_state_count + 1), -1);
  const auto state_index = [kDirectionCount](int cell, int direction) {
      return cell * kDirectionCount + direction;
    };
  const double turn_penalty = std::max(0.5, 4.0 * config_.minimum_turning_radius);
  const auto ranked_cost = [turn_penalty](int turn_count, double distance_cost) {
      return turn_penalty * static_cast<double>(turn_count) + distance_cost;
    };
  const auto turn_lower_bound = [goal_cell, &column_steps, &row_steps](
      int column, int row, int direction) {
      const int delta_column = goal_cell.first - column;
      const int delta_row = goal_cell.second - row;
      if (delta_column == 0 && delta_row == 0) {
        return 0;
      }
      if (delta_column != 0 && delta_row != 0) {
        const bool follows_needed_axis =
          (column_steps[direction] != 0 &&
          ((column_steps[direction] > 0) == (delta_column > 0))) ||
          (row_steps[direction] != 0 &&
          ((row_steps[direction] > 0) == (delta_row > 0)));
        return follows_needed_axis ? 1 : 2;
      }
      const int needed_column = delta_column == 0 ? 0 : (delta_column > 0 ? 1 : -1);
      const int needed_row = delta_row == 0 ? 0 : (delta_row > 0 ? 1 : -1);
      return column_steps[direction] == needed_column &&
             row_steps[direction] == needed_row ? 0 : 1;
    };
  turns[start_state] = 0;
  costs[start_state] = 0.0;
  // The special start state has no incoming direction and therefore no pivot
  // charge. The navigator deals with the initial heading before translation.
  frontier.emplace(0.0, 0, 0.0, start_state);
  int goal_state = -1;
  while (!frontier.empty()) {
    const auto [priority, current_turns, current_cost, state] = frontier.top();
    (void)priority;
    frontier.pop();
    if (current_turns != turns[state] ||
      current_cost > costs[state] + 1.0e-12)
    {
      continue;
    }
    const int current_direction = state == start_state ? kNoDirection : state % kDirectionCount;
    const int current_index = state == start_state ? start_index : state / kDirectionCount;
    const int column = current_index % columns_;
    const int row = current_index / columns_;
    if (current_index == goal_index) {
      goal_state = state;
      break;
    }
    for (int next_direction = 0; next_direction < kDirectionCount; ++next_direction) {
      const int next_column = column + column_steps[next_direction];
      const int next_row = row + row_steps[next_direction];
      if (!CellIsFree(next_column, next_row)) {
        continue;
      }
      int added_turns = 0;
      if (current_direction == kNoDirection && heading_constrained) {
        const double departure = direction_yaws[next_direction];
        if (!departure_is_allowed(departure)) {
          continue;
        }
        added_turns = initial_turn_count(departure);
      } else if (current_direction != kNoDirection &&
        current_direction != next_direction)
      {
        const Point pivot = CellToWorld({column, row});
        if (config_.turns_at_junctions_only &&
          !IsTurnJunction(pivot, config_.resolution * 0.75))
        {
          continue;
        }
        added_turns = (current_direction + 2) % kDirectionCount == next_direction ? 2 : 1;
        const bool is_reversal = added_turns == 2;
        if (is_reversal) {
          continue;
        }
        if ((is_reversal || !config_.allow_in_place_turns) &&
          !(config_.turns_at_junctions_only &&
          IsTurnJunction(pivot, config_.resolution * 0.75)) && !RotationIsFree(
            pivot, direction_yaws[current_direction],
            direction_yaws[next_direction]))
        {
          continue;
        }
      }
      // BuildGrid already inflates occupied cells by the chassis translation
      // radius plus half a grid cell. Cardinal motion between adjacent free
      // cell centres is therefore valid for search. MaterializeGridPath and
      // the public service still perform the full rectangular-footprint check
      // on every emitted control segment before it can reach navigation.
      const int next_cell_index = Index(next_column, next_row);
      const int next_state = state_index(next_cell_index, next_direction);
      const double clearance = clearance_[next_cell_index];
      const double deficit = config_.preferred_clearance > 1.0e-9 ?
        std::max(0.0, config_.preferred_clearance - clearance) /
        config_.preferred_clearance : 0.0;
      const double step = config_.resolution *
        (1.0 + config_.clearance_cost_weight * deficit * deficit);
      const int candidate_turns = current_turns + added_turns;
      const double candidate_cost = current_cost + step;
      if (ranked_cost(candidate_turns, candidate_cost) + 1.0e-12 >=
        ranked_cost(turns[next_state], costs[next_state]))
      {
        continue;
      }
      turns[next_state] = candidate_turns;
      costs[next_state] = candidate_cost;
      parents[next_state] = state;
      const int remaining_turns = turn_lower_bound(
        next_column, next_row, next_direction);
      const double remaining_distance = config_.resolution *
        (std::abs(goal_cell.first - next_column) +
        std::abs(goal_cell.second - next_row));
      frontier.emplace(
        ranked_cost(
          candidate_turns + remaining_turns,
          candidate_cost + remaining_distance),
        candidate_turns, candidate_cost, next_state);
    }
  }
  if (goal_state < 0) {
    throw std::runtime_error("no collision-free path exists");
  }
  std::vector<Cell> cells;
  int current = goal_state;
  while (current != start_state) {
    const int cell = current / kDirectionCount;
    cells.emplace_back(cell % columns_, cell / columns_);
    current = parents[current];
    if (current < 0) {
      throw std::runtime_error("A* predecessor chain is incomplete");
    }
  }
  cells.push_back(start_cell);
  std::reverse(cells.begin(), cells.end());
  GridPath result{
    cells, turns[goal_state], costs[goal_state], heading_constrained};
  return cache_path(std::move(result));
}
}  // namespace arena_path_planner
