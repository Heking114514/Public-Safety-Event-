#ifndef VISUAL_NAVIGATION__NAVIGATION_TURN_STATE_HPP_
#define VISUAL_NAVIGATION__NAVIGATION_TURN_STATE_HPP_

namespace visual_navigation
{

class NavigationTurnState
{
public:
  void reset_control()
  {
    rotating_ = false;
    waypoint_recovery_ = false;
    anchor_valid_ = false;
  }

  void reset_segment()
  {
    reset_control();
    path_alignment_completed_ = false;
  }

  void begin_rotation(double x, double y)
  {
    rotating_ = true;
    anchor_x_ = x;
    anchor_y_ = y;
    anchor_valid_ = true;
  }

  void ensure_anchor(double x, double y)
  {
    if (!anchor_valid_)
      begin_rotation(x, y);
  }

  void begin_waypoint_recovery() {waypoint_recovery_ = true;}
  void finish_waypoint_recovery() {waypoint_recovery_ = false;}

  void mark_path_aligned() {path_alignment_completed_ = true;}

  bool rotating() const {return rotating_;}
  bool waypoint_recovery() const {return waypoint_recovery_;}
  bool anchor_valid() const {return anchor_valid_;}
  bool path_alignment_completed() const {return path_alignment_completed_;}
  double anchor_x() const {return anchor_x_;}
  double anchor_y() const {return anchor_y_;}

private:
  double anchor_x_{0.0};
  double anchor_y_{0.0};
  bool path_alignment_completed_{false};
  bool rotating_{false};
  bool waypoint_recovery_{false};
  bool anchor_valid_{false};
};

}  // namespace visual_navigation

#endif  // VISUAL_NAVIGATION__NAVIGATION_TURN_STATE_HPP_
