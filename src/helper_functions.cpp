#include "helper_functions.hpp"

double GetBearing(Eigen::Vector2d dir, double obs_heading) {
  double heading_v = atan2(dir.y(), dir.x()); // error for (0,0)
  double theta = M_PI + obs_heading - heading_v; //Interception angle
  return remainder(theta, 2 * M_PI);
}

Eigen::Vector2d GetProjectionInObsFrame(const Eigen::Vector2d &point, Obstacle &obs, double time) {
  Eigen::Vector2d element_obs = point - ComputePosition(obs, time);
  Eigen::Rotation2D<double> rotation(obs.heading);
  return rotation.inverse() * element_obs;
}


