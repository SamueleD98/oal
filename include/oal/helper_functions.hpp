#ifndef HELPER_HPP
#define HELPER_HPP

#include "data_structs/misc.hpp"
#include "data_structs/node.hpp"
#include "data_structs/obstacle.hpp"
#include <set>

template<typename T>
Eigen::Vector2d ComputePosition(T &element, double time) {
  Eigen::Vector2d shift(element.speed * time * cos(element.heading), element.speed * time * sin(element.heading));
  return element.position + shift;
}

Eigen::Vector2d GetProjectionInObsFrame(const Eigen::Vector2d& point, Obstacle& obs, double time);

// Given a direction and a Target Ship heading, returns the approaching angle of TS wrt to Own Ship
double GetBearing(Eigen::Vector2d direction, double obs_heading);

#endif