#pragma once

#include <cstddef>
#include <deque>
#include <optional>
#include <string>
#include <vector>

namespace navigation_mission_manager
{

struct EventData
{
  std::string candidate_id;
  std::string source;
  std::string event_type;
  double confidence{0.0};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  std::string evidence_uri;
  std::string announcement;
  int attempts{0};
};

class EventRegistry
{
public:
  EventRegistry(double confidence_threshold, double merge_distance);

  bool offer(const EventData & event);
  const EventData * current() const;
  EventData mark_reported();
  EventData mark_attempt();
  void reset();

  std::size_t pending_count() const {return pending_.size();}
  std::size_t reported_count() const {return reported_.size();}

private:
  bool valid(const EventData & event) const;
  bool duplicate(const EventData & event) const;

  double confidence_threshold_;
  double merge_distance_;
  std::deque<EventData> pending_;
  std::vector<EventData> reported_;
};

}  // namespace navigation_mission_manager
