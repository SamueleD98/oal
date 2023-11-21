#include "data_structs/obstacle.hpp"
#include "helper_functions.hpp"

void Obstacle::SetSize(double dist_x, double dist_y, bool isAhead, double &bb_dim_x_stern, double &bb_dim_x_bow,
                       double &bb_dim_y) const {
  double bb_ratio_x;
  if (isAhead) {
    bb_dim_x_stern = dim_x / 2 * bb_x_stern_ratio;
    bb_ratio_x = bb_x_bow_ratio;
  } else {
    bb_dim_x_bow = dim_x / 2 * bb_x_bow_ratio;
    bb_ratio_x = bb_x_stern_ratio;
  }
  bool x_safety = (dist_x <= dim_x / 2 * safety_bb_ratio);
  bool y_safety = (dist_y <= dim_y / 2 * safety_bb_ratio);
  bool x_max = (dist_x >= dim_x / 2 * bb_ratio_x);
  bool y_max = (dist_y >= dim_y / 2 * bb_y_ratio);
  bool x_between = !x_safety && !x_max;
  bool y_between = !y_safety && !y_max;
  double bb_dim_x = dim_x / 2 * bb_ratio_x;
  bb_dim_y = dim_y * bb_y_ratio;
  if (x_safety && !y_max) {
    bb_dim_x = dim_x / 2 * safety_bb_ratio;
  }
  if (y_safety && !x_max) {
    bb_dim_y = dim_y * safety_bb_ratio;
  }
  if (y_between && !x_max) {
    bb_dim_y = dist_y * 2;
  }
  if (x_between && !y_max) {
    bb_dim_x = dist_x;
  }
  if (isAhead) {
    bb_dim_x_bow = bb_dim_x;
  } else {
    bb_dim_x_stern = bb_dim_x;
  }
}

void Obstacle::FindAbsVxs(double time, std::vector<Vertex> &vxs_abs) {
  Eigen::Vector2d current_obs_position = ComputePosition(*this, time);
  for (const Vertex &vx: vxs) {
    Vertex vx_abs;
    vx_abs.id = vx.id;
    Eigen::Rotation2D<double> rotation(head);
    vx_abs.position = current_obs_position + rotation * vx.position;
    vxs_abs.push_back(vx_abs);
  }
}

void Obstacle::FindLocalVxs(const Eigen::Vector2d &vhPos) {
  // Distance obs-vehicle wrt obs frame, on x and y
  Eigen::Vector2d bodyObs_vhPos = GetProjectionInObsFrame(vhPos, *this, 0);

  double dist_x = abs(bodyObs_vhPos.x());
  double dist_y = abs(bodyObs_vhPos.y());
  double bb_dim_y, bb_dim_x_bow, bb_dim_x_stern;
  // asymmetric bb x dimension computation
  double theta = atan2(bodyObs_vhPos.y(), bodyObs_vhPos.x()); // error for (0,0)
  // choose depending on approaching angle
  bool IsAhead = (abs(theta) <= M_PI / 2);
  SetSize(dist_x, dist_y, IsAhead, bb_dim_x_stern, bb_dim_x_bow, bb_dim_y);
  // Find the local vertexes position
  Vertex vx1, vx2, vx3, vx4;
  vx1.id = FR;
  vx1.position[0] = bb_dim_x_bow;
  vx1.position[1] = -bb_dim_y / 2;
  vx2.id = FL;
  vx2.position[0] = bb_dim_x_bow;
  vx2.position[1] = bb_dim_y / 2;
  vx3.id = RR;
  vx3.position[0] = -bb_dim_x_stern;
  vx3.position[1] = -bb_dim_y / 2;
  vx4.id = RL;
  vx4.position[0] = -bb_dim_x_stern;
  vx4.position[1] = bb_dim_y / 2;
  vxs.push_back(vx1);
  vxs.push_back(vx2);
  vxs.push_back(vx3);
  vxs.push_back(vx4);
}

Eigen::Vector2d Obstacle::GetProjectionInLocalFrame(TPoint &time_point) {
  Eigen::Vector2d element_obs = time_point.pos - ComputePosition(*this, time_point.time);
  Eigen::Rotation2D<double> rotation(head);
  return rotation.inverse() * element_obs;
}

bool Obstacle::IsInBB(TPoint &time_point) {
  Eigen::Vector2d bodyObs_element = GetProjectionInLocalFrame(time_point);

  // asymmetric bb x dimension computation
  double theta = atan2(bodyObs_element.y(), bodyObs_element.x()); // Approaching angle, error for (0,0)
  // choose comparing dimension based on theta
  bool IsAhead = (abs(theta) <= M_PI / 2);
  if (IsAhead) {
    return (abs(bodyObs_element.x()) < abs(vxs[0].position.x()) &&
            abs(bodyObs_element.y()) < abs(vxs[0].position.y()));
  } else {
    return (abs(bodyObs_element.x()) < abs(vxs[2].position.x()) &&
            abs(bodyObs_element.y()) < abs(vxs[2].position.y()));
  }
}

std::string Obstacle::plotStuff(double time) {
  std::ostringstream stream;
  stream << "Obs_" << id << std::endl;
  Eigen::Vector2d position = ComputePosition(*this, time);
  stream << "Position_" << position.x() << "_" << position.y() << std::endl;
  stream << "Heading_" << head << std::endl;
  stream << "Vel_" << vel_dir << std::endl;
  stream << "Dimx_" << dim_x << std::endl;
  stream << "Dimy_" << dim_y << std::endl;
  stream << "Safety_" << safety_bb_ratio << std::endl;
  std::vector<Vertex> vxs_abs;
  FindAbsVxs(time, vxs_abs);
  for (Vertex &vx: vxs_abs) {
    stream << "Vx_" << vx.position.x() << "_" << vx.position.y() << std::endl;
  }
  //std::cout << vxs_abs[wp.vx].position.x() << " " << vxs_abs[wp.vx].position.y() << " _ "  << std::endl;
  stream << "-" << std::endl;
  return stream.str();
}
