#include "minimum_jerk_trajectory_planner/robot.hpp"

namespace minimum_jerk
{

Robot::Robot(std::string name, double total_time, const TrajectoryPlanner &path_finder_controller, const Pose &pose_start, const Pose &pose_target, std::string move_type) : name_(name), total_time_(total_time), move_type_(move_type)
{
  this->path_finder_controller_ = std::make_unique<TrajectoryPlanner>(path_finder_controller);
  this->pose_ = std::make_unique<Pose>(pose_start);
  this->pose_start_ = std::make_unique<Pose>(pose_start);
  this->pose_target_ = std::make_unique<Pose>(pose_target);
  current_step_ = 0;
  is_at_target_ = false;

  if (move_type == "r" && move_type == "tx" && move_type == "ty") {
    throw "Couldn't build Robot instance, move type has to be 'r' or 'tx' or 'ty' (rotate or translate)";
  }
}

Robot::Robot(const Robot &src) : name_(src.get_name()), total_time_(src.get_total_time()), current_step_(src.get_current_step()), is_at_target_(src.get_is_at_target()), move_type_(src.get_move_type())
{
  this->path_finder_controller_ = std::make_unique<TrajectoryPlanner>(src.get_path_finder_controller());
  this->pose_ = std::make_unique<Pose>(src.get_pose());
  this->pose_start_ = std::make_unique<Pose>(src.get_pose_start());
  this->pose_target_ = std::make_unique<Pose>(src.get_pose_target());
  this->odometry_ = std::make_unique<Trajectory>(src.get_odometry());
}
Robot &Robot::operator=(const Robot &src)
{
  name_ = src.get_name();
  total_time_ = src.get_total_time();
  current_step_ = src.get_current_step();
  is_at_target_ = src.get_is_at_target();
  move_type_ = src.get_move_type();
  this->path_finder_controller_ = std::make_unique<TrajectoryPlanner>(src.get_path_finder_controller());
  this->pose_ = std::make_unique<Pose>(src.get_pose());
  this->pose_start_ = std::make_unique<Pose>(src.get_pose_start());
  this->pose_target_ = std::make_unique<Pose>(src.get_pose_target());
  this->odometry_ = std::make_unique<Trajectory>(src.get_odometry());
  return *this;
}

void Robot::generate_trajectory()
{
  path_finder_controller_->generate_trajectory(this->get_pose_start(), this->get_pose_target());
  if (!path_finder_controller_->get_list_timestamps().empty()) {
    odometry_ = std::make_unique<Trajectory>(path_finder_controller_->get_list_timestamps(), path_finder_controller_->get_list_poses());
  }
}
bool Robot::operator==(const Robot &r) const
{
  return name_ == r.get_name() && pose_start_->get_x() == r.get_pose_start().get_x() && pose_start_->get_y() == r.get_pose_start().get_y() && pose_start_->get_theta() == r.get_pose_start().get_theta();
}
bool Robot::operator!=(const Robot &r) const
{
  return !(*this == r);
}
Pose Robot::get_pose() const
{
  return Pose(*pose_);
}
Pose Robot::get_pose_start() const
{
  return Pose(*pose_start_);
}
Pose Robot::get_pose_target() const
{
  return Pose(*pose_target_);
}
std::string Robot::get_move_type() const
{
  return move_type_;
}
TrajectoryPlanner Robot::get_path_finder_controller() const
{
  return TrajectoryPlanner(*path_finder_controller_);
}
std::string Robot::get_name() const
{
  return name_;
}
double Robot::get_total_time() const
{
  return total_time_;
}
int Robot::get_current_step() const
{
  return current_step_;
}
bool Robot::get_is_at_target() const
{
  return is_at_target_;
}
Trajectory Robot::get_odometry() const
{
  return Trajectory(*odometry_);
}
}