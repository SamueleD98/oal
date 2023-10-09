#include "path_planner.hpp"

// Compute the path to reach the goal
bool path_planner::ComputePath(const Eigen::Vector2d &goal_position, bool colregs, std::stack<Node> &waypoints) {
  colregs_compliance = colregs;
  // Plot stuff
  plotWpsFile_ << "Start_" << v_info_.position(0) << "_" << v_info_.position(1) << std::endl;
  plotWpsFile_ << "Goal_" << goal_position(0) << "_" << goal_position(1) << std::endl;

  std::multiset<Node> open_set;  // set of viable nodes, ordered by total cost (ascending)
  std::multiset<Node> close_set; // set of not viable nodes
  Node root, goal, current;

  // The planner starts from the vehicle position with time=0
  root.position = v_info_.position;
  root.time = 0;
  //UpdateCosts(root, goal_position, v_info_.speed);
  root.UpdateCosts(goal_position, v_info_.speed);
  open_set.insert(root);

  goal.position = goal_position;

  // Set the bounding box dimension based on the vehicle start distance from the obstacles
  for (Obstacle &obs: obss_info_.obstacles) {
    obs.ComputeLocalVertexes(v_info_.position);
  }

  /*std::vector<Obstacle> surrounding_obs;
  if(IsInAnyBB(root.position, 0, surrounding_obs)){
    // The starting point is in a number of different bb
    std::vector<vx_id> allowedVxs;
    for (Obstacle &obs: surrounding_obs){
      FindExitVxs(root.position, obs, 0, allowedVxs);
    }
  }*/

  for (Obstacle &obs: obss_info_.obstacles) {
    if (IsInBB(root.position, obs, 0)) {
      std::vector<vx_id> allowedVxs;
      FindExitVxs(root.position, obs, 0, allowedVxs);
      // Look for the best vx between the two now available, depending on the optimal heading
      //  (if vehicle actual heading is later added as attribute, may be better than optimal heading)
      Eigen::Vector2d path = goal_position - v_info_.position;
      double theta = atan2(path.y(), path.x());
      double bearingVx1 =
              theta - (atan2(obs.vxs[allowedVxs[0]].position.y(), obs.vxs[allowedVxs[0]].position.x()) + obs.heading);
      double bearingVx2 =
              theta - (atan2(obs.vxs[allowedVxs[1]].position.y(), obs.vxs[allowedVxs[1]].position.x()) + obs.heading);
      vx_id exit_vx_id;
      if (abs(bearingVx1) < abs(bearingVx2)) {
        exit_vx_id = allowedVxs[0];
      } else {
        exit_vx_id = allowedVxs[1];
      }
      // If usv is able to intercept it, add the exit vx as next (and only) node
      std::vector<Vertex> vxs_abs;
      // Find vxs in abs frame
      FindAbsVxs(obs, 0, vxs_abs);
      vxs_abs[exit_vx_id].isVisible = true;
      // Compute when every visible vertex is intercepted
      FindInterceptPoints(root.position, obs, vxs_abs);
      if (!vxs_abs.empty()) {
        Vertex vx = vxs_abs[0];
        // Save new node
        Node newstart;
        newstart.position = vx.ip_position;
        newstart.obs = obs.id;
        newstart.obs_heading = obs.heading;
        newstart.vx = vx.id;
        newstart.time = 0 + vx.ip_time;
        //std::cout << vx.ip_time << std::endl;
        //UpdateCosts(newstart, goal_position, v_info_.speed);
        newstart.UpdateCosts(goal_position, v_info_.speed);
        std::shared_ptr<Node> parent = std::make_shared<Node>(root);
        newstart.parent = parent;
        // Remove root and add the first waypoint out of the bb
        open_set.erase(open_set.begin());
        open_set.insert(newstart);
      }
    }
  }


  // Run until every node is studied or a path is found
  while (!open_set.empty()) {
    current = *open_set.begin();
    open_set.erase(open_set.begin());


    if (CheckFinal(current, goal)) {
      BuildPath(&goal, waypoints);
      return true;
    }

    for (Obstacle &obs: obss_info_.obstacles) {
      std::vector<Vertex> vxs_abs;
      // Find vertexes in abs frame
      FindAbsVxs(obs, current.time, vxs_abs);
      // Find vxs visibility
      FindVisibility(current, obs, vxs_abs);
      // Compute when every visible vertex is intercepted
      FindInterceptPoints(current.position, obs, vxs_abs);
      // Check collision for every one of them
      for (const Vertex &vx: vxs_abs) {
        // Create new Node (still to check for collision)
        Node new_node;
        new_node.position = vx.ip_position;
        new_node.obs = obs.id;
        new_node.obs_heading = obs.heading;
        new_node.vx = vx.id;
        new_node.time = current.time + vx.ip_time;

        new_node.colregsLimitedVxs = {};
        if (CheckColreg(current, new_node)) {
          // Does respect colregs or their compliance is not asked for
          if (CheckCollision(current, new_node)) {
            // Collision free
            if (IsIn(new_node, open_set)) {
              // Loop detected, do not expand from this node
              close_set.insert(new_node);
              continue;
            }
            if (IsIn(new_node, close_set)) {
              // Avoid loop
              continue;
            }
            // Save new node
            //UpdateCosts(new_node, goal_position, v_info_.speed);
            new_node.UpdateCosts(goal_position, v_info_.speed);
            std::shared_ptr<Node> parent = std::make_shared<Node>(current);
            new_node.parent = parent;
            open_set.insert(new_node);
          }
        }
      }
    }
  }
  // No path found
  return false;
}

bool path_planner::CheckFinal(const Node &start, Node &goal) {
  std::vector<std::string> surrounding_obs;
  double goal_time = start.time + (goal.position - start.position).norm() / v_info_.speed;
  if (IsInAnyBB(goal.position, goal_time, &surrounding_obs)) {
    // the goal is unreachable, let's run check collision to get the collision points
    std::vector<Node> collision_nodes;
    CheckCollision(start, goal, &collision_nodes);
    // The newgoal might be the closer collision point iff it is a surrounding obs
    // otherwise there's an actual collision before getting to that goal
    // closer collision, maybe move it in checkCollision
    Node closer_collision = goal;
    double min_distance = closer_collision.position.norm();
    for (const auto &node: collision_nodes) {
      Eigen::Vector2d point(node.position.x(), node.position.y());
      if (point.norm() < min_distance) {
        closer_collision = node;
        min_distance = closer_collision.position.norm();
      }
    }
    if (std::count(surrounding_obs.begin(), surrounding_obs.end(), closer_collision.obs)) {
      goal = closer_collision;
      std::cout << "Trying different goal!" << std::endl;
    } else {
      return false;
    }
  }
  if (CheckCollision(start, goal)) {
    // no colregs yet, but assuming it needs to check colregs only when approaching an obs,
    //  goal has no obs and so it must not check
    // ACTUALLY PAST SENTENCE DOES NOT HOLD IF GOAL WAS MADE IN THIS FUNCTION
    Eigen::Vector2d dist_to_goal = goal.position - start.position;
    goal.time = start.time + dist_to_goal.norm() / v_info_.speed;
    std::shared_ptr<Node> parent = std::make_shared<Node>(start);
    goal.parent = parent;
    return true;
  }
  return false;
}

bool path_planner::CheckColreg(const Node &start, Node &goal) const {
  // Check only if colregs compliance is requested
  if (!colregs_compliance) {
    return true;
  }
  // Check only if moving between different obstacles
  if (start.obs == goal.obs) {
    // otherwise is compliant iff goal vx is not limited because of past maneuver
    for (const vx_id &vx: start.colregsLimitedVxs) {
      if (vx == goal.vx) return false;
    }
    return true;
  }

  // Get the vehicle desired heading
  Eigen::Vector2d path = goal.position - start.position;
  double heading_v = atan2(path[1], path[0]); // error for (0,0)
  if (heading_v < 0) heading_v += 2 * M_PI;

  // Interception angle
  double theta = goal.obs_heading + heading_v;
  //std::cout << theta << std::endl;
  if (theta > M_PI) theta -= 2 * M_PI;

  if (abs(theta) <= 15 * (M_PI / 180)) {
    // head on
    if (goal.vx == FR) {
      // should be FL
      return false;
    }
    goal.colregsLimitedVxs.push_back(FR);
  }
  if (theta > 15 * (M_PI / 180) && theta < 112 * (M_PI / 180)) {
    // crossing from right
    if (goal.vx == RR || goal.vx == RL) {
      // should stand on (does it make sense to ignore completely obs that should give way?)
      return false;
    }
  }
  if (theta < -15 * (M_PI / 180) && theta > -112 * (M_PI / 180)) {
    // crossing from left
    if (goal.vx == FR || goal.vx == FL) {
      // should give way
      return false;
    }
    goal.colregsLimitedVxs.push_back(FR);
    goal.colregsLimitedVxs.push_back(FL);
  }
  return true;
}

// Check if the path between start and goal collide with any obstacle
bool path_planner::CheckCollision(Node start, Node &goal, std::vector<Node> *collision_points) {
  Eigen::Vector3d start3d = {start.position[0], start.position[1], 0};
  Eigen::Vector3d goal3d = {goal.position[0], goal.position[1], 0};
  // vehicle path to check for collision
  Eigen::Vector3d path = goal3d - start3d;
  // time from start to goal
  goal3d[2] = path.norm() / v_info_.speed;
  //path = goal3d - start3d;

  // plot
  plotCKFile_ << "---" << std::endl;
  plotCKFile_ << "Start_" << start3d(0) << "_" << start3d(1) << "_" << start3d(2) << std::endl;
  plotCKFile_ << "Goal_" << goal3d(0) << "_" << goal3d(1) << "_" << goal3d(2) << std::endl;

  for (Obstacle obs: obss_info_.obstacles) {
    // Compute obstacle direction in x-y-t
    Eigen::Vector3d bb_direction(obs.speed * cos(obs.heading), obs.speed * sin(obs.heading), 1);
    //bb_direction.normalize(); no, because multiplied by t* it returns the position at time t*

    // Find vertexes in abs frame
    std::vector<Vertex> vxs_abs;
    FindAbsVxs(obs, start.time, vxs_abs);

    plotCKFile_ << "Obs_" << obs.id << std::endl;

    // Uncomment to plot
    //if(start.obs == "1" && start.obs == goal.obs) {
    //if(start.obs=="1" && goal.obs == "" && start.vx == RR){
    //  plotCKFile_ << "PlotIt" << std::endl;
    //}

    plotCKFile_ << "Direction_" << bb_direction(0) << "_" << bb_direction(1) << "_" << bb_direction(2) << std::endl;
    for (Vertex vx: vxs_abs) {
      plotCKFile_ << "Vx_" << vx.position[0] << "_" << vx.position[1] << std::endl;
    }
    plotCKFile_ << "-" << std::endl;

    // if the goal obstacle is this one, then do not check, the algorithm already aims for the visible vxs
    // (this assumption does not hold with overlapping bounding boxes)
    if (obs.id == goal.obs) {
      continue;
    }

    // for each of the bb 4 sides
    for (int side_idx = 0; side_idx < 4; side_idx++) {
      int vx_idx1, vx_idx2;
      switch (side_idx) {
        case 0:
          vx_idx1 = 0;
          vx_idx2 = 2;
          break;
        case 1:
          vx_idx1 = 0;
          vx_idx2 = 1;
          break;
        case 2:
          vx_idx1 = 3;
          vx_idx2 = 2;
          break;
        case 3:
          vx_idx1 = 3;
          vx_idx2 = 1;
          break;
        default:
          continue;
      }

      // if the path starts from the current obs, do not check the sides of the departing vx
      if (obs.id == start.obs && (start.vx == vx_idx1 || start.vx == vx_idx2)) {
        continue;
      }
      /*blindly jump the check here can cause collision when the goal is on another bb side that is then not incident with the path
      resulting in false negative outcome and so a collision because the current side is traversed by own ship
      */

      Eigen::Vector3d bb_vector1(vxs_abs[vx_idx1].position[0], vxs_abs[vx_idx1].position[1], 0);
      Eigen::Vector3d bb_vector2(vxs_abs[vx_idx2].position[0], vxs_abs[vx_idx2].position[1], 0);

      Eigen::Vector3d collision_point;
      if (FindLinePlaneIntersectionPoint(vxs_abs[vx_idx1], vxs_abs[vx_idx2], bb_direction, start3d, goal3d,
                                         v_info_.speed, collision_point)) {
        // The desired path crosses the face of the bb defined by the two vxs and its direction
        Node cp_node;
        Eigen::Vector2d collision_point_2d(collision_point[0], collision_point[1]);
        cp_node.position = collision_point_2d;
        cp_node.obs = obs.id;
        cp_node.time = start.time + collision_point[2];
        if (collision_points != nullptr) (*collision_points).push_back(cp_node);
        return false;
      }
    }
    // done checking the sides
    if (obs.id == start.obs) {
      // check the opposite diagonal wrt the start vx
      int vx_idx1, vx_idx2;
      if (start.vx == FL || start.vx == RR) {
        vx_idx1 = FR;
        vx_idx2 = RL;
      } else {
        vx_idx1 = FL;
        vx_idx2 = RR;
      }
      Eigen::Vector3d bb_vector1(vxs_abs[vx_idx1].position[0], vxs_abs[vx_idx1].position[1], 0);
      Eigen::Vector3d bb_vector2(vxs_abs[vx_idx2].position[0], vxs_abs[vx_idx2].position[1], 0);

      Eigen::Vector3d collision_point;
      if (FindLinePlaneIntersectionPoint(vxs_abs[vx_idx1], vxs_abs[vx_idx2], bb_direction, start3d, goal3d,
                                         v_info_.speed, collision_point)) {
        return false;
      }
    }
  }
  plotCKFile_ << "Good" << std::endl;
  return true;
}

bool path_planner::FindLinePlaneIntersectionPoint(Vertex vx1, Vertex vx2, const Eigen::Vector3d &bb_direction,
                                                  const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
                                                  double speed, Eigen::Vector3d &collision_point) {
  Eigen::Vector3d path = goal - start;
  Eigen::Vector3d vx1_pos(vx1.position[0], vx1.position[1], 0);
  Eigen::Vector3d vx2_pos(vx2.position[0], vx2.position[1], 0);

  // Get the plane normal
  Eigen::Vector3d side = vx1_pos - vx2_pos;
  Eigen::Vector3d planeNormal = bb_direction.cross(side);
  planeNormal.normalize();

  // if the plane and the path are //
  if (planeNormal.dot(path) == 0) return false; //do we need tolerance here?

  // Compute position along the path where the collision happens (from http://paulbourke.net/geometry/pointlineplane/)
  Eigen::Vector3d startToOne = vx1_pos - start;
  double c_pos = planeNormal.dot(startToOne) / planeNormal.dot(path);

  // collision_pos has to be between 0 and 1 (start and goal) to be an actual problem

  /*double tolerance = 10; //meters
  Eigen::Vector2d distance(path.x(), path.y());
  double tol_percent = tolerance / distance.norm();*/

  if (c_pos >= 0 && c_pos <= 1) { // intersect_point between start and goal
    // what about u==1 ? means the collision happens when reaching the vertex,
    //  that makes sense when approaching the obstacle I'm aiming to

    // Get collision time and point
    Eigen::Vector3d doable_path = path * c_pos; // Part of the path prior the collision
    doable_path[2] = 0;
    double collision_time = doable_path.norm() / speed; // When the collision happens
    doable_path[2] = collision_time;
    collision_point = start + doable_path; // Where the collision happens

    // Get vertexes at time t*
    Eigen::Vector3d vertex1_position = vx1_pos + bb_direction * collision_time;
    Eigen::Vector3d vertex2_position = vx2_pos + bb_direction * collision_time;

    // Check if point is inside those two vertexes
    Eigen::Vector3d P1 = collision_point - vertex1_position;
    Eigen::Vector3d P2 = collision_point - vertex2_position;


    if (P1.dot(P2) <= 0) return true;
  }
  return false;
}

void path_planner::BuildPath(Node *it, std::stack<Node> &waypoints) {    // Build path from goal to initial position
  Node wp;

  while (it->parent != nullptr) {
    wp.position = it->position;
    wp.time = it->time;
    wp.vx = it->vx;
    // std::cout << wp.position.x() << " " << wp.position.y() << " _ " << wp.time << std::endl;
    waypoints.push(wp);
    it = &*it->parent;

    //  Plot stuff
    plotWpsFile_ << "Time_" << wp.time << std::endl;
    plotWpsFile_ << "Waypoint_" << wp.position(0) << "_" << wp.position(1) << std::endl;
    for (Obstacle &obs: obss_info_.obstacles) {
      plotWpsFile_ << "Obs_" << obs.id << std::endl;
      Eigen::Vector2d position = ComputePosition(obs, wp.time);
      plotWpsFile_ << "Position_" << position(0) << "_" << position(1) << std::endl;
      plotWpsFile_ << "Heading_" << obs.heading << std::endl;
      plotWpsFile_ << "Dimx_" << obs.dim_x << std::endl;
      plotWpsFile_ << "Dimy_" << obs.dim_y << std::endl;
      plotWpsFile_ << "Safety_" << obs.safety_bb_ratio << std::endl;
      std::vector<Vertex> vxs_abs;
      FindAbsVxs(obs, wp.time, vxs_abs);
      for (Vertex &vx: vxs_abs) {
        plotWpsFile_ << "Vx_" << vx.position.x() << "_" << vx.position.y() << std::endl;
      }
      //std::cout << vxs_abs[wp.vx].position.x() << " " << vxs_abs[wp.vx].position.y() << " _ "  << std::endl;
      plotWpsFile_ << "-" << std::endl;
    }
  }

  // Plot stuff
  plotWpsFile_ << "Time_" << "0" << std::endl;
  plotWpsFile_ << "Waypoint_" << v_info_.position(0) << "_" << v_info_.position(1) << std::endl;
  for (Obstacle &obs: obss_info_.obstacles) {
    plotWpsFile_ << "Obs_" << obs.id << std::endl;
    Eigen::Vector2d position = ComputePosition(obs, 0);
    plotWpsFile_ << "Position_" << position(0) << "_" << position(1) << std::endl;
    plotWpsFile_ << "Heading_" << obs.heading << std::endl;
    plotWpsFile_ << "Dimx_" << obs.dim_x << std::endl;
    plotWpsFile_ << "Dimy_" << obs.dim_y << std::endl;
    plotWpsFile_ << "Safety_" << obs.safety_bb_ratio << std::endl;
    std::vector<Vertex> vxs_abs;
    FindAbsVxs(obs, 0, vxs_abs);
    for (Vertex &vx: vxs_abs) {
      plotWpsFile_ << "Vx_" << vx.position.x() << "_" << vx.position.y() << std::endl;
    }
    plotWpsFile_ << "-" << std::endl;
  }
}


// Given some obstacle vertexes, find the intercept points
void path_planner::FindInterceptPoints(const Eigen::Vector2d &vehicle_position, Obstacle &obstacle,
                                       std::vector<Vertex> &vxs_abs) const {
  for (auto it = vxs_abs.begin(); it != vxs_abs.end();/*nothing here*/) {
    if (it->isVisible) {
      // algorithm described in
      // 'A Three-Layered Architecture for Real Time Path Planning and Obstacle Avoidance for Surveillance USVs Operating in Harbour Fields'
      Eigen::Vector2d vertex_vehicle = it->position - vehicle_position;
      double k = atan2(-vertex_vehicle[1], vertex_vehicle[0]); // error for (0,0)

      // NORM OF OBS_SPEED HAS TO BE EQUAL OR SMALLER THAN V_SPEED
      // or, more complete, the argument of the asin has to be between -1 and +1
      if (abs(obstacle.speed * sin(obstacle.heading + k) / v_info_.speed) <= 1) {
        double theta = asin(obstacle.speed * sin(obstacle.heading + k) / v_info_.speed) - k;
        double t;
        if (abs(vertex_vehicle[0]) > 0.001) { // should it be something more than 0?
          t = vertex_vehicle[0] / (v_info_.speed * cos(theta) - obstacle.speed * cos(obstacle.heading));
        } else {
          t = vertex_vehicle[1] / (v_info_.speed * sin(theta) - obstacle.speed * sin(obstacle.heading));
        }

        Eigen::Vector2d intercept_point_position;
        intercept_point_position[0] = vehicle_position[0] + v_info_.speed * cos(theta) * t;
        intercept_point_position[1] = vehicle_position[1] + v_info_.speed * sin(theta) * t;

        // Check if point is in an obstacle bb => overlapping bbs, ignore the point
        std::vector<std::string> surrounding_obs;
        bool isInAnyBB = IsInAnyBB(intercept_point_position, t, &surrounding_obs);
        if (isInAnyBB && surrounding_obs.size() == 1 && surrounding_obs[0] == obstacle.id) isInAnyBB = false;

        // Save the point
        if (!isInAnyBB) {
          it->ip_position = intercept_point_position;
          it->ip_time = t;
          ++it;
          continue;
        }
      }
    }
    it = vxs_abs.erase(it);
  }


}

void path_planner::FindVisibility(Node &node, Obstacle &obs, std::vector<Vertex> &vxs_abs) {
  if (node.obs == obs.id) {
    // Set the adjacent vxs as visible
    if (node.vx == FR || node.vx == RL) {
      vxs_abs[1].isVisible = true;
      vxs_abs[2].isVisible = true;
    } else {
      vxs_abs[0].isVisible = true;
      vxs_abs[3].isVisible = true;
    }
  } else {
    // Find the angles between vectors from vehicle to each vxs and the vector from vehicle to the obs center,
    //  then the visible vxs are the ones with min/max angle + the ones closer to the vehicle than these two
    Eigen::Vector2d obs_position = ComputePosition(obs, node.time);
    Eigen::Vector2d obs_vh = obs_position - node.position;
    //obs_vh.normalize();

    std::vector<double> thetas;  // angles wrt abs frame
    std::vector<Eigen::Vector2d> vxs_vh;

    for (Vertex &vx: vxs_abs) {
      Eigen::Vector2d vx_vh = vx.position - node.position; //if 0 there's a problem
      //vx_vh.normalize();
      double sign =
              obs_vh[0] * vx_vh[1] - obs_vh[1] * vx_vh[0]; // same as z value of a cross product (and so its direction)
      double theta = sign / abs(sign) * acos(obs_vh.normalized().dot(vx_vh.normalized()));
      thetas.push_back(theta);
      vxs_vh.push_back(vx_vh);
    }
    // not good, check the kinda difference of the angles!!!!!!!!!!ppppppppppppppp
    auto max_v_ptr = std::max_element(thetas.begin(), thetas.end());
    auto max_v = std::distance(thetas.begin(), max_v_ptr);
    auto min_v_ptr = std::min_element(thetas.begin(), thetas.end());
    auto min_v = std::distance(thetas.begin(), min_v_ptr);
    // min/max angle vertexes are visible
    vxs_abs[min_v].isVisible = true;
    vxs_abs[max_v].isVisible = true;
    // are also visible vertexes which distance is smaller than those two
    double distance, min_v_distance, max_v_distance;
    min_v_distance = vxs_vh[min_v].norm();
    max_v_distance = vxs_vh[max_v].norm();
    for (size_t i = 0; i < vxs_vh.size(); ++i) {
      if (i == min_v || i == max_v) {
        continue;
      }
      distance = vxs_vh[i].norm();
      if (distance < min_v_distance && distance < max_v_distance) {
        vxs_abs[i].isVisible = true;
      }
    }
  }
}

bool path_planner::IsInAnyBB(const Eigen::Vector2d &element_pos, double time,
                             std::vector<std::string> *surrounding_obs) const {
  for (const Obstacle &obs: obss_info_.obstacles) {
    if (IsInBB(element_pos, obs, time)) {
      if (surrounding_obs != nullptr) {
        surrounding_obs->push_back(obs.id);
      }
      return true;
    }
  }
  return false;
}

bool path_planner::IsInBB(const Eigen::Vector2d &element_pos, Obstacle obs, double time) {
  Eigen::Vector2d element_obs = element_pos - ComputePosition(obs, time);
  Eigen::Rotation2D<double> rotation(obs.heading);
  Eigen::Vector2d bodyObs_element = rotation.inverse() * element_obs;

  // asymmetric bb x dimension computation
  double theta = atan2(bodyObs_element.y(), bodyObs_element.x()); // Approaching angle, error for (0,0)
  // choose comparing dimension based on theta
  bool IsAhead = (abs(theta) <= M_PI / 2);
  if (IsAhead) {
    return (abs(bodyObs_element.x()) < abs(obs.vxs[0].position.x()) &&
            abs(bodyObs_element.y()) < abs(obs.vxs[0].position.y()));
  } else {
    return (abs(bodyObs_element.x()) < abs(obs.vxs[2].position.x()) &&
            abs(bodyObs_element.y()) < abs(obs.vxs[2].position.y()));
  }
}

void path_planner::FindExitVxs(const Eigen::Vector2d &element_pos, Obstacle &obs, double time,
                               std::vector<vx_id> &allowedVxs) {
  Eigen::Vector2d element_obs = element_pos - ComputePosition(obs, time);
  Eigen::Rotation2D<double> rotation(obs.heading);
  Eigen::Vector2d bodyObs_e = rotation.inverse() * element_obs;

  bool diag1 = (bodyObs_e.y() >=
                obs.vxs[1].position.y() / obs.vxs[1].position.x() * bodyObs_e.x()); // Being left of diag FL-RR
  bool diag2 = (bodyObs_e.y() >=
                obs.vxs[0].position.y() / obs.vxs[0].position.x() * bodyObs_e.x()); // Being left of diag FR-LL

  if (diag1 && diag2) {
    // USV in port side of bb
    //  go for FL or RL
    allowedVxs.push_back(FL);
    allowedVxs.push_back(RL);
  } else {
    if (diag1) {
      // USV in stern side of bb
      //  go for RL or RR
      allowedVxs.push_back(RL);
      allowedVxs.push_back(RR);
    } else {
      if (diag2) {
        // USV in bow side of bb
        //  go for FR or FL
        allowedVxs.push_back(FR);
        allowedVxs.push_back(FL);
      } else {
        // USV in starboard side of bb
        //  go for FR or RR
        allowedVxs.push_back(FR);
        allowedVxs.push_back(RR);
      }
    }
  }
}
