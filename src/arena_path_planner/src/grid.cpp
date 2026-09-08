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

std::vector<int8_t> ArenaPlanner::OccupancyData() const
{
  std::vector<int8_t> result;
  result.reserve(occupied_.size());
  for (bool value : occupied_) {
    result.push_back(value ? 100 : 0);
  }
  return result;
}

ArenaPlanner::GridPath ArenaPlanner::AStar(const Point & start, const Point & goal) const
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
  using Entry = std::tuple<double, double, int, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> frontier;
  const int start_index = Index(start_cell.first, start_cell.second);
  const int goal_index = Index(goal_cell.first, goal_cell.second);
  std::vector<double> costs(columns_ * rows_, std::numeric_limits<double>::infinity());
  std::vector<int> parents(columns_ * rows_, -1);
  std::vector<bool> closed(columns_ * rows_, false);
  costs[start_index] = 0.0;
  frontier.emplace(Distance(start, goal), 0.0, start_cell.first, start_cell.second);
  const std::vector<std::tuple<int, int, double>> steps{
    {1, 0, 1.0}, {-1, 0, 1.0}, {0, 1, 1.0}, {0, -1, 1.0},
    {1, 1, std::sqrt(2.0)}, {1, -1, std::sqrt(2.0)},
    {-1, 1, std::sqrt(2.0)}, {-1, -1, std::sqrt(2.0)}};
  while (!frontier.empty()) {
    const auto [priority, current_cost, column, row] = frontier.top();
    (void)priority;
    frontier.pop();
    const int current_index = Index(column, row);
    if (closed[current_index]) {
      continue;
    }
    closed[current_index] = true;
    if (current_index == goal_index) {
      break;
    }
    for (const auto & [column_delta, row_delta, multiplier] : steps) {
      const int next_column = column + column_delta;
      const int next_row = row + row_delta;
      if (!CellIsFree(next_column, next_row)) {
        continue;
      }
      if (column_delta != 0 && row_delta != 0 &&
        (!CellIsFree(column + column_delta, row) ||
        !CellIsFree(column, row + row_delta)))
      {
        continue;
      }
      // The inflated grid is only a translational lower bound. Validate
      // diagonal grid legs with the oriented chassis before allowing them into
      // an A* predecessor chain; this rejects shortcuts that clip a corner
      // even when both endpoint cells are individually free. Cardinal legs
      // are covered by the inflation bound and avoid an expensive resampling
      // pass for every A* expansion.
      if (column_delta != 0 && row_delta != 0 &&
        !SegmentIsFree(
          CellToWorld({column, row}), CellToWorld({next_column, next_row})))
      {
        continue;
      }
      const int next_index = Index(next_column, next_row);
      const double clearance = clearance_[next_index];
      const double deficit = config_.preferred_clearance > 1.0e-9 ?
        std::max(0.0, config_.preferred_clearance - clearance) /
        config_.preferred_clearance : 0.0;
      const double step = multiplier * config_.resolution *
        (1.0 + config_.clearance_cost_weight * deficit * deficit);
      const double candidate = current_cost + step;
      if (candidate + 1.0e-12 >= costs[next_index]) {
        continue;
      }
      costs[next_index] = candidate;
      parents[next_index] = current_index;
      const Point next_point = CellToWorld({next_column, next_row});
      frontier.emplace(
        candidate + Distance(next_point, goal), candidate, next_column, next_row);
    }
  }
  if (!std::isfinite(costs[goal_index])) {
    throw std::runtime_error("no collision-free path exists");
  }
  std::vector<Cell> cells;
  int current = goal_index;
  while (current != start_index) {
    cells.emplace_back(current % columns_, current / columns_);
    current = parents[current];
    if (current < 0) {
      throw std::runtime_error("A* predecessor chain is incomplete");
    }
  }
  cells.push_back(start_cell);
  std::reverse(cells.begin(), cells.end());
  return {cells, costs[goal_index]};
}
}  // namespace arena_path_planner
