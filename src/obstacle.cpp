#include "data_structs/obstacle.hpp"

void Obstacle::SetSize(double dist_x, double dist_y, double dim_x, double dim_y, double safety_ratio, double bb_ratio_x,
                  double bb_ratio_y, double &bb_dim_x, double &bb_dim_y) {
  bool x_safety = (dist_x <= dim_x / 2 * safety_ratio);
  bool y_safety = (dist_y <= dim_y / 2 * safety_ratio);
  bool x_max = (dist_x >= dim_x / 2 * bb_ratio_x);
  bool y_max = (dist_y >= dim_y / 2 * bb_ratio_y);
  bool x_between = !x_safety && !x_max;
  bool y_between = !y_safety && !y_max;

  bb_dim_x = dim_x / 2 * bb_ratio_x;
  bb_dim_y = dim_y * bb_ratio_y;

  if (x_safety && !y_max) {
    bb_dim_x = dim_x / 2 * safety_ratio;
  }
  if (y_safety && !x_max) {
    bb_dim_y = dim_y * safety_ratio;
  }
  if (y_between && !x_max) {
    bb_dim_y = dist_y * 2;
  }
  if (x_between && !y_max) {
    bb_dim_y = dist_x;
  }
}

void Obstacle::ComputeLocalVertexes(const Eigen::Vector2d &vhPos) {
  // Distance obs-vehicle wrt obs frame, on x and y
  Eigen::Vector2d vh_obs = vhPos - position;
  Eigen::Rotation2D<double> rotation(heading);
  Eigen::Vector2d bodyObs_vhPos = rotation.inverse() * vh_obs;
  double dist_x = abs(bodyObs_vhPos.x());
  double dist_y = abs(bodyObs_vhPos.y());
  double bb_dim_y, bb_dim_x_bow, bb_dim_x_stern;
  // asymmetric bb x dimension computation
  double theta = atan2(bodyObs_vhPos.y(), bodyObs_vhPos.x()); // error for (0,0)
  // choose depending on approaching angle
  bool IsAhead = (abs(theta) <= M_PI / 2);
  if (IsAhead) {
    bb_dim_x_stern = dim_x / 2 * bb_x_stern_ratio;
    SetSize(dist_x, dist_y, dim_x, dim_y, safety_bb_ratio, bb_x_bow_ratio, bb_y_ratio, bb_dim_x_bow, bb_dim_y);
  } else {
    bb_dim_x_bow = dim_x / 2 * bb_x_bow_ratio;
    SetSize(dist_x, dist_y, dim_x, dim_y, safety_bb_ratio, bb_x_stern_ratio, bb_y_ratio, bb_dim_x_stern, bb_dim_y);
  }
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
