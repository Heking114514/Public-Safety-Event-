#include "arena_path_planner/planner_internal.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace arena_path_planner
{
namespace
{

bool AxisAligned(const Point & left, const Point & right)
{
  constexpr double tolerance = 1.0e-9;
  return std::abs(left.x - right.x) <= tolerance ||
         std::abs(left.y - right.y) <= tolerance;
}

}  // namespace

bool ArenaPlanner::IsPlannedReverseRetreat(
  const std::vector<Point> & points, std::size_t pivot_index) const
{
  if (pivot_index == 0 || pivot_index + 1 >= points.size()) {
    return false;
  }
  const Point & segment_start = points[pivot_index - 1];
  const Point & endpoint = points[pivot_index];
  const Point & retreat_target = points[pivot_index + 1];
  return Distance(segment_start, retreat_target) <= kPlannedRetreatReturnTolerance &&
         Distance(segment_start, endpoint) >= kPlannedRetreatMinimumLength &&
         SegmentIsFree(endpoint, retreat_target);
}

std::vector<Point> ArenaPlanner::StopAfterFirstBlockedReversal(
  const std::vector<Point> & points, bool & inserted_retreat) const
{
  inserted_retreat = false;
  if (points.size() < 3) {
    return points;
  }

  std::vector<Point> result{points.front()};
  if (Distance(points[0], points[1]) <= 1.0e-9) {
    result.push_back(points[1]);
    return result;
  }
  result.push_back(points[1]);
  double vehicle_heading = std::atan2(
    points[1].y - points[0].y,
    points[1].x - points[0].x);

  for (std::size_t index = 1; index + 1 < points.size(); ++index) {
    const double leg_length = Distance(points[index], points[index + 1]);
    if (leg_length <= 1.0e-9) {
      if (Distance(result.back(), points[index + 1]) > 1.0e-9) {
        result.push_back(points[index + 1]);
      }
      continue;
    }

    const double outgoing = std::atan2(
      points[index + 1].y - points[index].y,
      points[index + 1].x - points[index].x);
    if (IsPlannedReverseRetreat(points, index)) {
      if (Distance(result.back(), points[index + 1]) > 1.0e-9) {
        result.push_back(points[index + 1]);
      }
      inserted_retreat = true;
      return result;
    }

    if (std::abs(NormalizeAngle(outgoing - vehicle_heading)) >=
      kNearReversalThreshold)
    {
      const Point & retreat_target = points[index - 1];
      if (Distance(retreat_target, points[index]) >= kPlannedRetreatMinimumLength &&
        SegmentIsFree(points[index], retreat_target))
      {
        if (Distance(result.back(), retreat_target) > 1.0e-9) {
          result.push_back(retreat_target);
        }
        inserted_retreat = true;
        return result;
      }
    }

    if (Distance(result.back(), points[index + 1]) > 1.0e-9) {
      result.push_back(points[index + 1]);
    }
    vehicle_heading = outgoing;
  }
  return result;
}

std::vector<Point> ArenaPlanner::Simplify(
  const std::vector<Point> & points,
  const std::vector<Point> & required_targets) const
{
  if (points.size() < 2 || required_targets.empty()) {
    if (points.size() <= 2) {
      return points;
    }
  }

  // A long straight connector can pass through an inspection intersection
  // without the grid path containing that exact floating-point coordinate.
  // Insert such semantic points before simplification so they remain usable
  // as reverse/replan anchors after a dynamic obstacle is detected.
  std::vector<Point> anchored_points;
  anchored_points.reserve(points.size() + required_targets.size());
  if (!points.empty()) {
    anchored_points.push_back(points.front());
  }
  for (std::size_t index = 1; index < points.size(); ++index) {
    const Point & begin = points[index - 1];
    const Point & end = points[index];
    const double dx = end.x - begin.x;
    const double dy = end.y - begin.y;
    const double squared_length = dx * dx + dy * dy;
    std::vector<std::pair<double, Point>> interior;
    if (squared_length > 1.0e-12) {
      for (const Point & target : required_targets) {
        if (DistanceToSegment(target, begin, end) > 1.0e-6) {
          continue;
        }
        const double ratio =
          ((target.x - begin.x) * dx + (target.y - begin.y) * dy) /
          squared_length;
        if (ratio > 1.0e-6 && ratio < 1.0 - 1.0e-6) {
          interior.emplace_back(ratio, target);
        }
      }
    }
    std::sort(
      interior.begin(), interior.end(),
      [](const auto & left, const auto & right) {return left.first < right.first;});
    for (const auto & [ratio, target] : interior) {
      (void)ratio;
      if (Distance(anchored_points.back(), target) > 1.0e-9) {
        anchored_points.push_back(target);
      }
    }
    if (anchored_points.empty() || Distance(anchored_points.back(), end) > 1.0e-9) {
      anchored_points.push_back(end);
    }
  }
  if (anchored_points.size() <= 2) {
    return anchored_points;
  }
  const std::vector<Point> & candidates = anchored_points;
  std::vector<Point> result{candidates.front()};
  std::size_t index = 0;
  while (index + 1 < candidates.size()) {
    std::size_t candidate = candidates.size() - 1;
    for (std::size_t required_index = index + 1;
      required_index < candidates.size(); ++required_index)
    {
      const bool required = std::any_of(
        required_targets.begin(), required_targets.end(),
        [&candidates, required_index](const Point & target) {
          return Distance(target, candidates[required_index]) <= 1.0e-6;
        });
      if (required) {
        candidate = required_index;
        break;
      }
    }
    while (candidate > index + 1) {
      // Do not replace a rectilinear grid chain with a diagonal line-of-sight
      // shortcut. Long cardinal legs give the controller one stable heading;
      // a diagonal is retained only when it already exists as an unavoidable
      // one-segment connection to an arbitrary start or target.
      if (!AxisAligned(candidates[index], candidates[candidate])) {
        --candidate;
        continue;
      }
      const double shortcut_heading = std::atan2(
        candidates[candidate].y - candidates[index].y,
        candidates[candidate].x - candidates[index].x);
      bool creates_blocked_reversal = false;
      if (result.size() >= 2) {
        const double incoming_heading = std::atan2(
          result.back().y - result[result.size() - 2].y,
          result.back().x - result[result.size() - 2].x);
        creates_blocked_reversal =
          std::abs(NormalizeAngle(shortcut_heading - incoming_heading)) > 2.6;
      }
      if (!creates_blocked_reversal && candidate + 1 < candidates.size()) {
        const double outgoing_heading = std::atan2(
          candidates[candidate + 1].y - candidates[candidate].y,
          candidates[candidate + 1].x - candidates[candidate].x);
        creates_blocked_reversal =
          std::abs(NormalizeAngle(outgoing_heading - shortcut_heading)) > 2.6;
      }
      if (creates_blocked_reversal) {
        --candidate;
        continue;
      }
      const double required = std::max(
        0.0, std::min(
          {config_.preferred_clearance, Clearance(candidates[index]),
            Clearance(candidates[candidate])}) - config_.resolution * 0.25);
      bool clearance_ok = true;
      const double length = Distance(candidates[index], candidates[candidate]);
      const int samples = std::max(
        1, static_cast<int>(std::ceil(length / config_.resolution)));
      for (int sample = 0; sample <= samples && clearance_ok; ++sample) {
        const double ratio = static_cast<double>(sample) / samples;
        const Point point{
          candidates[index].x + ratio * (candidates[candidate].x - candidates[index].x),
          candidates[index].y + ratio * (candidates[candidate].y - candidates[index].y)};
        clearance_ok = Clearance(point) + 1.0e-9 >= required;
      }
      if (SegmentIsFree(candidates[index], candidates[candidate]) && clearance_ok) {
        break;
      }
      --candidate;
    }
    result.push_back(candidates[candidate]);
    index = candidate;
  }
  return result;
}

std::vector<Point> ArenaPlanner::MaterializeGridPath(
  const GridPath & path, const Point & exact_start, const Point & exact_end) const
{
  if (path.cells.empty()) {
    throw std::runtime_error("grid path has no cells");
  }
  std::vector<Point> grid_points;
  grid_points.reserve(path.cells.size());
  for (const Cell & cell : path.cells) {
    grid_points.push_back(CellToWorld(cell));
  }

  // YAML road centres commonly have the same sub-cell offset at both ends.
  // Translate the complete cardinal chain by that shared offset instead of
  // creating two tiny elbow pairs merely to enter and leave cell centres.
  const Point offset{
    exact_start.x - grid_points.front().x,
    exact_start.y - grid_points.front().y};
  std::vector<Point> translated;
  translated.reserve(grid_points.size());
  for (const Point & point : grid_points) {
    translated.push_back({point.x + offset.x, point.y + offset.y});
  }
  if (Distance(translated.back(), exact_end) <= 1.0e-6) {
    std::vector<Point> required{exact_end};
    if (path.preserve_initial_direction && translated.size() >= 2) {
      required.push_back(translated[1]);
    }
    translated = Simplify(translated, required);
    const bool translated_is_free = std::all_of(
      translated.begin() + 1, translated.end(),
      [this, &translated, index = std::size_t{0}](const Point &) mutable {
        const bool free = SegmentIsFree(translated[index], translated[index + 1]);
        ++index;
        return free;
      });
    if (translated_is_free) {
      return translated;
    }
  }

  std::vector<Point> result{exact_start};
  const auto append_connection = [this](
      std::vector<Point> & output, const Point & end) {
      const Point begin = output.back();
      if (Distance(begin, end) <= 1.0e-9) {
        return true;
      }
      if (AxisAligned(begin, end) && SegmentIsFree(begin, end)) {
        output.push_back(end);
        return true;
      }
      // Prefer an orthogonal two-leg connector so exact points on grid-cell
      // boundaries do not introduce tiny diagonal headings.
      for (const Point & elbow :
        {Point{end.x, begin.y}, Point{begin.x, end.y}})
      {
        if (Distance(begin, elbow) > 1.0e-9 &&
          !SegmentIsFree(begin, elbow))
        {
          continue;
        }
        if (Distance(elbow, end) > 1.0e-9 && !SegmentIsFree(elbow, end)) {
          continue;
        }
        if (Distance(begin, elbow) > 1.0e-9) {
          output.push_back(elbow);
        }
        if (Distance(output.back(), end) > 1.0e-9) {
          output.push_back(end);
        }
        return true;
      }
      // A non-cardinal configured inspection stroke is still a valid explicit
      // requirement. Keep that direct leg only when the full footprint fits.
      if (SegmentIsFree(begin, end)) {
        output.push_back(end);
        return true;
      }
      return false;
    };

  std::size_t first = 0;
  std::vector<Point> prefix;
  for (; first < grid_points.size(); ++first) {
    prefix = result;
    if (append_connection(prefix, grid_points[first])) {
      break;
    }
  }
  if (first == grid_points.size()) {
    throw std::runtime_error("grid path cannot leave the exact start point");
  }
  result = std::move(prefix);
  for (std::size_t index = first + 1; index < grid_points.size(); ++index) {
    if (Distance(result.back(), grid_points[index]) > 1.0e-9) {
      result.push_back(grid_points[index]);
    }
  }
  if (!append_connection(result, exact_end)) {
    throw std::runtime_error("grid path cannot reach the exact end point");
  }

  std::vector<Point> required{exact_end};
  if (path.preserve_initial_direction && result.size() >= 2) {
    required.push_back(result[1]);
  }
  result = Simplify(result, required);
  for (std::size_t index = 1; index < result.size(); ++index) {
    if (!SegmentIsFree(result[index - 1], result[index])) {
      throw std::runtime_error("materialized grid path is not collision-free");
    }
  }
  return result;
}

std::vector<int> ArenaPlanner::VisitOrder(
  const std::vector<std::vector<double>> & distances, const std::string & mode) const
{
  const int task_count = static_cast<int>(distances.size()) - 1;
  if (mode == "numbered") {
    std::vector<int> result;
    for (int index = 0; index <= task_count; ++index) {
      result.push_back(index);
    }
    result.push_back(0);
    return result;
  }
  if (mode != "shortest") {
    throw std::runtime_error("mode must be shortest or numbered");
  }
  const int mask_count = 1 << task_count;
  std::vector<double> costs(mask_count * task_count, std::numeric_limits<double>::infinity());
  std::vector<int> parents(mask_count * task_count, -1);
  const auto key = [task_count](int mask, int last) {return mask * task_count + last;};
  for (int task = 0; task < task_count; ++task) {
    costs[key(1 << task, task)] = distances[0][task + 1];
  }
  for (int mask = 1; mask < mask_count; ++mask) {
    for (int last = 0; last < task_count; ++last) {
      if (!(mask & (1 << last))) {
        continue;
      }
      const int previous_mask = mask ^ (1 << last);
      if (previous_mask == 0) {
        continue;
      }
      for (int previous = 0; previous < task_count; ++previous) {
        if (!(previous_mask & (1 << previous))) {
          continue;
        }
        const double candidate = costs[key(previous_mask, previous)] +
          distances[previous + 1][last + 1];
        if (candidate < costs[key(mask, last)]) {
          costs[key(mask, last)] = candidate;
          parents[key(mask, last)] = previous;
        }
      }
    }
  }
  const int full_mask = mask_count - 1;
  int last = 0;
  double best = std::numeric_limits<double>::infinity();
  for (int task = 0; task < task_count; ++task) {
    const double candidate = costs[key(full_mask, task)] + distances[task + 1][0];
    if (candidate < best) {
      best = candidate;
      last = task;
    }
  }
  std::vector<int> reversed;
  int mask = full_mask;
  while (mask != 0) {
    reversed.push_back(last + 1);
    const int previous = parents[key(mask, last)];
    mask ^= 1 << last;
    last = previous;
  }
  std::reverse(reversed.begin(), reversed.end());
  reversed.insert(reversed.begin(), 0);
  reversed.push_back(0);
  return reversed;
}

std::vector<Point> ArenaPlanner::Smooth(
  const std::vector<Point> & points, const std::vector<Point> & required_targets) const
{
  if (points.size() < 2) {
    return points;
  }
  std::vector<Point> aligned = points;
  if (config_.turns_at_junctions_only) {
    const auto align_axis = [this](double coordinate, bool horizontal) {
        double closest = coordinate;
        double error = config_.resolution * 0.75;
        const auto consider = [&](const Point & junction) {
            const double axis = horizontal ? junction.x : junction.y;
            if (std::abs(coordinate - axis) < error) {
              closest = axis;
              error = std::abs(coordinate - axis);
            }
          };
        for (const Point & junction : config_.inspection_nodes) consider(junction);
        for (const Point & junction : config_.turn_junctions) consider(junction);
        return closest;
      };
    for (std::size_t index = 1; index + 1 < aligned.size(); ++index) {
      aligned[index].x = align_axis(aligned[index].x, true);
      aligned[index].y = align_axis(aligned[index].y, false);
    }
    bool removed_spur = true;
    while (removed_spur && aligned.size() >= 3) {
      removed_spur = false;
      for (std::size_t index = 1; index + 1 < aligned.size(); ++index) {
        if (IsPlannedReverseRetreat(aligned, index)) {
          continue;
        }
        if (Distance(aligned[index - 1], aligned[index + 1]) <= 1.0e-6) {
          aligned.erase(aligned.begin() + static_cast<std::ptrdiff_t>(index),
                        aligned.begin() + static_cast<std::ptrdiff_t>(index + 2));
          removed_spur = true;
          break;
        }
      }
    }
    for (std::size_t index = 1; index + 1 < aligned.size(); ++index) {
      const Point original = aligned[index];
      if (std::any_of(required_targets.begin(), required_targets.end(),
        [&original](const Point & target) {return Distance(original, target) <= 1.0e-6;}))
      {
        continue;
      }
      const auto snap = [this, &aligned, index, &original](const Point & junction) {
          if (Distance(original, junction) > config_.resolution * 0.75) {
            return false;
          }
          aligned[index] = junction;
          return true;
        };
      if (std::any_of(config_.inspection_nodes.begin(), config_.inspection_nodes.end(), snap)) {
        continue;
      }
      std::any_of(config_.turn_junctions.begin(), config_.turn_junctions.end(), snap);
    }
  }
  bool removed_duplicate = true;
  while (removed_duplicate && aligned.size() >= 2) {
    removed_duplicate = false;
    for (std::size_t index = 1; index < aligned.size(); ++index) {
      if (Distance(aligned[index - 1], aligned[index]) <= 1.0e-6) {
        aligned.erase(aligned.begin() + static_cast<std::ptrdiff_t>(index));
        removed_duplicate = true;
        break;
      }
    }
  }
  // A* connectors and consecutive coverage layers can share their endpoint.
  // Remove those zero-length legs before computing headings; otherwise the
  // next segment appears as an artificial 180 degree turn to the navigator.
  std::vector<Point> clean;
  clean.reserve(aligned.size());
  for (const Point & point : aligned) {
    if (clean.empty() || Distance(clean.back(), point) > 1.0e-6) {
      clean.push_back(point);
    }
  }
  // A connector can end one grid cell before a layer endpoint and the next
  // connector can immediately return to that same cell. Collapse these tiny
  // out-and-back artifacts before reducing the control polyline.
  bool collapsed = true;
  while (collapsed && clean.size() >= 3) {
    collapsed = false;
    for (std::size_t index = 1; index + 1 < clean.size(); ++index) {
      const bool required = std::any_of(
        required_targets.begin(), required_targets.end(),
        [&clean, index](const Point & target) {
          return Distance(target, clean[index]) <= 1.0e-6;
        });
      if (required) {
        continue;
      }
      if (IsPlannedReverseRetreat(clean, index)) {
        continue;
      }
      const double joined_heading = std::atan2(
        clean[index + 1].y - clean[index - 1].y,
        clean[index + 1].x - clean[index - 1].x);
      const double outgoing_heading = std::atan2(
        clean[index + 1].y - clean[index].y,
        clean[index + 1].x - clean[index].x);
      // A live replan starts from the fused pose, which is normally a few
      // millimetres off the nominal road centre. Do not turn that harmless
      // offset into a tiny perpendicular leg followed by a fake 90 degree
      // turn. Join it into the first forward leg when the resulting heading
      // change remains below the normal path-turn threshold.
      const bool small_start_alignment =
        index == 1 &&
        Distance(clean[index - 1], clean[index]) <=
        kTurnJunctionOperatingTolerance &&
        std::abs(NormalizeAngle(joined_heading - outgoing_heading)) <
        config_.in_place_turn_heading_threshold &&
        SegmentIsFree(clean[index - 1], clean[index + 1]);
      if (Distance(clean[index - 1], clean[index + 1]) <= config_.resolution * 0.15 ||
        small_start_alignment ||
        (config_.turns_at_junctions_only &&
        (Distance(clean[index - 1], clean[index]) <= config_.resolution * 0.75 ||
        Distance(clean[index], clean[index + 1]) <= config_.resolution * 0.75) &&
        AxisAligned(clean[index - 1], clean[index + 1]) &&
        SegmentIsFree(clean[index - 1], clean[index + 1])))
      {
        clean.erase(clean.begin() + static_cast<std::ptrdiff_t>(index));
        collapsed = true;
        break;
      }
    }
  }
  // The controller now receives only semantic control points: start, required
  // target/road crossings, actual corners, and end. It interpolates along each
  // long straight itself; adding samples here merely causes repeated waypoint
  // switching and PID resets without changing the geometry.
  std::vector<Point> result = Simplify(clean, required_targets);
  result.erase(
    std::unique(result.begin(), result.end(), [](const Point & left, const Point & right) {
      return Distance(left, right) <= 1.0e-4;
    }), result.end());
  return result;
}
}  // namespace arena_path_planner
