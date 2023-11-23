#ifndef HELPER_HPP
#define HELPER_HPP

#include "oal/data_structs/misc.hpp"
#include "oal/data_structs/obstacle.hpp"
#include <set>

template<typename T>
Eigen::Vector2d ComputePosition(T &element, double time) {
  Eigen::Vector2d shift(element.speed * time * cos(element.vel_dir), element.speed * time * sin(element.vel_dir));
  return element.position + shift;
}

template<typename T>
Eigen::Vector3d Get3dPos(T &element){
  return {element.position.x(), element.position.y(), 0};
}

Eigen::Vector2d GetProjectionInObsFrame(const Eigen::Vector2d &point, Obstacle &obs, double time);

// Given a direction and a Target Ship heading, returns the approaching angle of TS wrt to Own Ship
double GetBearing(Eigen::Vector2d direction, double obs_heading);

#endif