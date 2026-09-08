#include "arena_path_planner/planner_internal.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace arena_path_planner
{
std::vector<Point> ArenaPlanner::Simplify(
  const std::vector<Point> & points,
  const std::vector<Point> & required_targets) const
{
  if (points.size() <= 2) {
    return points;
  }
  std::vector<Point> result{points.front()};
  std::size_t index = 0;
  while (index + 1 < points.size()) {
    std::size_t candidate = points.size() - 1;
    for (std::size_t required_index = index + 1;
      required_index < points.size(); ++required_index)
    {
      const bool required = std::any_of(
        required_targets.begin(), required_targets.end(),
        [&points, required_index](const Point & target) {
          return Distance(target, points[required_index]) <= 1.0e-6;
        });
      if (required) {
        candidate = required_index;
        break;
      }
    }
    while (candidate > index + 1) {
      const double required = std::max(
        0.0, std::min(
          {config_.preferred_clearance, Clearance(points[index]),
            Clearance(points[candidate])}) - config_.resolution * 0.25);
      bool clearance_ok = true;
      const double length = Distance(points[index], points[candidate]);
      const int samples = std::max(
        1, static_cast<int>(std::ceil(length / config_.resolution)));
      for (int sample = 0; sample <= samples && clearance_ok; ++sample) {
        const double ratio = static_cast<double>(sample) / samples;
        const Point point{
          points[index].x + ratio * (points[candidate].x - points[index].x),
          points[index].y + ratio * (points[candidate].y - points[index].y)};
        clearance_ok = Clearance(point) + 1.0e-9 >= required;
      }
      if (SegmentIsFree(points[index], points[candidate]) && clearance_ok) {
        break;
      }
      --candidate;
    }
    result.push_back(points[candidate]);
    index = candidate;
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
  // A* connectors and consecutive coverage layers can share their endpoint.
  // Remove those zero-length legs before computing headings; otherwise the
  // next segment appears as an artificial 180 degree turn to the navigator.
  std::vector<Point> clean;
  clean.reserve(points.size());
  for (const Point & point : points) {
    if (clean.empty() || Distance(clean.back(), point) > 1.0e-6) {
      clean.push_back(point);
    }
  }
  // A connector can end one grid cell before a layer endpoint and the next
  // connector can immediately return to that same cell. Collapse these tiny
  // out-and-back artifacts; they otherwise become a visible spike and a
  // 180-degree heading sample despite being only a few millimetres long.
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
      if (Distance(clean[index - 1], clean[index + 1]) <= config_.resolution * 0.15) {
        clean.erase(clean.begin() + static_cast<std::ptrdiff_t>(index));
        collapsed = true;
        break;
      }
    }
  }
  if (clean.size() < 3) {
    return clean;
  }
  std::vector<Point> output{clean.front()};
  const auto append_line = [this, &output](const Point & end) {
      const Point start = output.back();
      const double length = Distance(start, end);
      const int count = std::max(
        1, static_cast<int>(std::ceil(length / config_.waypoint_spacing)));
      for (int sample = 1; sample <= count; ++sample) {
        const double ratio = static_cast<double>(sample) / count;
        output.push_back(
          {start.x + ratio * (end.x - start.x),
            start.y + ratio * (end.y - start.y)});
      }
    };
  for (std::size_t index = 1; index + 1 < clean.size(); ++index) {
    const Point previous = clean[index - 1];
    const Point corner = clean[index];
    const Point following = clean[index + 1];
    const double incoming_length = Distance(previous, corner);
    const double outgoing_length = Distance(corner, following);
    if (incoming_length <= 1.0e-9 || outgoing_length <= 1.0e-9) {
      append_line(corner);
      continue;
    }
    const Point incoming{
      (corner.x - previous.x) / incoming_length,
      (corner.y - previous.y) / incoming_length};
    const Point outgoing{
      (following.x - corner.x) / outgoing_length,
      (following.y - corner.y) / outgoing_length};
    const double turn = std::abs(NormalizeAngle(
        std::atan2(outgoing.y, outgoing.x) - std::atan2(incoming.y, incoming.x)));
    // A near-U-turn is a genuine constrained reversal, not a corner to round
    // with a quadratic curve. Bezier interpolation there doubles back over
    // the entry point and produces an artificial spike in the route.
    if (turn < config_.maximum_heading_step || turn > 2.6) {
      append_line(corner);
      continue;
    }
    // A curved transition sweeps the chassis through intermediate headings.
    // In a narrow lane the centre-line chord may be free while that swept
    // rectangle is not. Keep an axis-aligned corner in that case; the
    // navigator can apply the configured in-place-turn policy at the corner.
    if (!RotationIsFree(
        corner, std::atan2(incoming.y, incoming.x), std::atan2(outgoing.y, outgoing.x))) {
      append_line(corner);
      continue;
    }
    const double maximum_trim = std::min(
      {config_.minimum_turning_radius, incoming_length * 0.35,
        outgoing_length * 0.35});
    bool accepted = false;
    for (double scale : {1.0, 0.85, 0.7, 0.55, 0.4}) {
      const double trim = maximum_trim * scale;
      if (trim < config_.curve_spacing) {
        continue;
      }
      const Point entry{corner.x - incoming.x * trim, corner.y - incoming.y * trim};
      const Point exit{corner.x + outgoing.x * trim, corner.y + outgoing.y * trim};
      const int dense_count = std::max(
        8, static_cast<int>(std::ceil(2.0 * trim / (config_.resolution * 0.5))));
      std::vector<Point> dense;
      for (int sample = 0; sample <= dense_count; ++sample) {
        dense.push_back(QuadraticBezier(
            entry, corner, exit, static_cast<double>(sample) / dense_count));
      }
      bool free = SegmentIsFree(output.back(), entry);
      for (std::size_t sample = 1; sample < dense.size() && free; ++sample) {
        free = SegmentIsFree(dense[sample - 1], dense[sample]);
      }
      bool preserves_targets = true;
      for (const Point & target : required_targets) {
        if (Distance(target, corner) > 1.0e-6) {
          continue;
        }
        double minimum = std::numeric_limits<double>::infinity();
        for (std::size_t sample = 1; sample < dense.size(); ++sample) {
          minimum = std::min(
            minimum, DistanceToSegment(target, dense[sample - 1], dense[sample]));
        }
        preserves_targets = minimum <= config_.task_tolerance;
      }
      if (!free || !preserves_targets) {
        continue;
      }
      append_line(entry);
      const int curve_count = std::max(
        {2, static_cast<int>(std::ceil(turn / config_.maximum_heading_step)),
          static_cast<int>(std::ceil(2.0 * trim / config_.curve_spacing))});
      for (int sample = 1; sample <= curve_count; ++sample) {
        output.push_back(QuadraticBezier(
            entry, corner, exit, static_cast<double>(sample) / curve_count));
      }
      accepted = true;
      break;
    }
    if (!accepted) {
      append_line(corner);
    }
  }
  append_line(clean.back());
  std::vector<Point> deduplicated;
  deduplicated.reserve(output.size());
  for (const Point & point : output) {
    if (deduplicated.empty() || Distance(deduplicated.back(), point) > 1.0e-6) {
      deduplicated.push_back(point);
    }
  }
  output = std::move(deduplicated);
  bool free = true;
  for (std::size_t index = 1; index < output.size(); ++index) {
    if (!SegmentIsFree(output[index - 1], output[index])) {
      free = false;
      break;
    }
  }
  if (!free) {
    // A single tight corner must not discard smoothing for the whole route.
    // Simplify retains collision-free line-of-sight sections and leaves only
    // the locally constrained corner as a short polyline.
    return Simplify(clean, required_targets);
  }
  return output;
}
}  // namespace arena_path_planner
