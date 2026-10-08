#include "navigation_mission_manager/events.hpp"

#include <cmath>
#include <stdexcept>

namespace navigation_mission_manager
{

EventRegistry::EventRegistry(double confidence_threshold, double merge_distance)
: confidence_threshold_(confidence_threshold), merge_distance_(merge_distance)
{
}

bool EventRegistry::offer(const EventData & event)
{
  if (!valid(event) || duplicate(event)) {
    return false;
  }
  pending_.push_back(event);
  return true;
}

const EventData * EventRegistry::current() const
{
  return pending_.empty() ? nullptr : &pending_.front();
}

EventData EventRegistry::mark_reported()
{
  if (pending_.empty()) {
    throw std::runtime_error("no pending event to confirm");
  }
  EventData event = pending_.front();
  pending_.pop_front();
  reported_.push_back(event);
  return event;
}

EventData EventRegistry::mark_attempt()
{
  if (pending_.empty()) {
    throw std::runtime_error("no pending event to retry");
  }
  ++pending_.front().attempts;
  return pending_.front();
}

void EventRegistry::reset()
{
  pending_.clear();
  reported_.clear();
}

bool EventRegistry::valid(const EventData & event) const
{
  return !event.event_type.empty() && std::isfinite(event.confidence) &&
         event.confidence >= confidence_threshold_ && std::isfinite(event.x) &&
         std::isfinite(event.y) && std::isfinite(event.yaw);
}

bool EventRegistry::duplicate(const EventData & event) const
{
  const auto matches = [this, &event](const EventData & existing) {
      if (!event.candidate_id.empty() && event.candidate_id == existing.candidate_id) {
        return true;
      }
      return event.event_type == existing.event_type &&
             std::hypot(event.x - existing.x, event.y - existing.y) <= merge_distance_;
    };
  for (const auto & existing : pending_) {
    if (matches(existing)) {
      return true;
    }
  }
  for (const auto & existing : reported_) {
    if (matches(existing)) {
      return true;
    }
  }
  return false;
}

}  // namespace navigation_mission_manager
