#ifndef HELPER_HPP
#define HELPER_HPP

#include "data_structs.hpp"
#include <set>

template <typename T>
Eigen::Vector2d ComputePosition(T& element, double time)
{
  Eigen::Vector2d shift(element.speed * time * cos(element.heading), element.speed * time * sin(element.heading));
  return element.position + shift;
}

// Compute the costs of a newly created Node,
//  depending on the time to reach the last node and the goal position
void UpdateCosts(Node &node, const Eigen::Vector2d &goal, double speed);

// Find the vxs position in the absolute frame depending on the time instant
void ComputeAbsVertexes(Obstacle &obs, double time, std::vector<Vertex> &vxs_abs);

bool AlreadyExists(Node new_node, std::multiset<Node>& set);

#endif