#include "path_planner.hpp"

#include <utility>

// Compute the path to reach the goal
bool path_planner::ComputePath(const Eigen::Vector2d &goal_position, bool colregs, Path path) {
  noCrossList = path.overtakingObsList;
  colregs_compliance = colregs;

  // Plot stuff
  {
    plotWpsFile_ << "Start_" << v_info_.position(0) << "_" << v_info_.position(1) << std::endl;
    plotWpsFile_ << "Goal_" << goal_position(0) << "_" << goal_position(1) << std::endl;
  }

  std::multiset<Node> open_set;  // set of viable nodes, ordered by total cost (ascending)
  std::multiset<Node> close_set; // set of not viable nodes
  Node root, goal, current;

  // Root node setup
  root.position = v_info_.position;
  root.time = 0;
  root.UpdateCosts(goal_position, v_info_.speed);
  open_set.insert(root);
  // Goal node setup
  goal.position = goal_position;

  // Set the bounding box dimension based on the vehicle start distance from the obstacles
  for (const std::shared_ptr<Obstacle>& obs: obss_info_.obstacles) {
    obs->FindLocalVxs(v_info_.position);
  }

  // If starting inside bb
  std::shared_ptr<std::vector<obs_ptr>> surrounding_obs(new std::vector<obs_ptr>);
  if(IsInAnyBB(root.position, 0, surrounding_obs)){
    if(surrounding_obs->size()>1){
      // The starting point is in a number of different bb, at the moment no solutions
      return false;
    }
    std::shared_ptr<Obstacle> obs = surrounding_obs->at(0);
    std::vector<vx_id> allowedVxs;
    FindExitVxs(root.position, obs, 0, allowedVxs);
    // Look for the best vx between the two now available, depending on the optimal heading
    //  (if vehicle actual heading is later added as attribute, may be better than optimal heading)
    Eigen::Vector2d path_vector = goal_position - v_info_.position;
    double theta = atan2(path_vector.y(), path_vector.x());
    double bearingVx1 =
            theta - (atan2(obs->vxs[allowedVxs[0]].position.y(), obs->vxs[allowedVxs[0]].position.x()) + obs->heading);
    double bearingVx2 =
            theta - (atan2(obs->vxs[allowedVxs[1]].position.y(), obs->vxs[allowedVxs[1]].position.x()) + obs->heading);
    vx_id exit_vx_id;
    if (abs(bearingVx1) < abs(bearingVx2)) {
      exit_vx_id = allowedVxs[0];
    } else {
      exit_vx_id = allowedVxs[1];
    }
    // If usv is able to intercept it, add the exit vx as next (and only) node
    std::vector<Vertex> vxs_abs;
    obs->FindAbsVxs(0, vxs_abs);
    vxs_abs[exit_vx_id].isVisible = true;
    FindInterceptPoints(root.position, *obs, vxs_abs); //Compute when every visible vertex is intercepted
    if (!vxs_abs.empty()) {
      Vertex vx = vxs_abs[0];
      // Save new node
      Node newstart(vx.ip_position, obs, vx.id, 0+vx.ip_time);
      newstart.UpdateCosts(goal_position, v_info_.speed);
      // Creating a pointer (to root node) that will later be copied inside the opens_set
      std::shared_ptr<Node> parent = std::make_shared<Node>(root);
      newstart.parent = parent;
      // Remove root and add the first waypoint out of the bb
      open_set.erase(open_set.begin());
      open_set.insert(newstart);
    }
    // else ignore being in the bb, nothing it can do
  }

  // Run until every node is studied or a path is found
  while (!open_set.empty()) {
    current = *open_set.begin();
    open_set.erase(open_set.begin());

    // Check if goal is reachable
    if (CheckFinal(current, goal)) {
      std::stack<Node> waypoints;
      BuildPath(goal, waypoints);
      path.waypoints = std::move(waypoints);
      path.overtakingObsList = current.overtakingObsList;
      //std::cout << "count: " << current.parent.use_count() << std::endl;
      return true;
    }

    for (const obs_ptr& obs_ptr: obss_info_.obstacles) {
      Obstacle obs = *obs_ptr.get();
      std::vector<Vertex> reachable_vxs;
      obs.FindAbsVxs(current.time, reachable_vxs); //Find vertexes in abs frame
      FindVisibility(current, obs, reachable_vxs); //Find vxs visibility
      FindInterceptPoints(current.position, obs, reachable_vxs); //Compute when every visible vertex is intercepted
      // Check collision for each of the visible, intercept-able, vxs
      for (const Vertex &vx: reachable_vxs) {
        // Create new Node (still to check for collision)
        Node new_node(vx.ip_position, obs_ptr, vx.id, current.time+vx.ip_time);
        new_node.overtakingObsList = current.overtakingObsList;
        if (CheckColreg(current, new_node)) {
          // Does respect colregs or their compliance is not asked for
          if (CheckCollision(current, new_node)) {
            // Collision free
            if (new_node.IsIn(open_set)) {
              // Loop detected, do not expand from this node
              close_set.insert(new_node);
              continue;
            }
            if (new_node.IsIn(close_set)) {
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
  //std::shared_ptr<std::vector<obs_ptr>> surrounding_obs(new std::vector<obs_ptr>);
  std::vector<obs_ptr> s_obs;
  double goal_time = start.time + (goal.position - start.position).norm() / v_info_.speed;
  if (IsInAnyBB(goal.position, goal_time, std::make_shared<std::vector<obs_ptr>>(s_obs))) {
    // the goal is unreachable, let's try and find a new goal.
    std::shared_ptr<std::vector<Node>> collision_nodes(new std::vector<Node>);
    CheckCollision(start, goal, collision_nodes); //run check collision to get the collision points
    // The new goal might be the closer collision point iff it is a surrounding obs
    // otherwise there's an actual collision before getting to that goal
    // closer collision, maybe move it in checkCollision
    Node closer_collision;
    start.GetCloser(collision_nodes, closer_collision);
    bool found_new = false;
    for (const auto& obs_ptr: s_obs){
      if(obs_ptr->id == closer_collision.obs){
        goal = closer_collision;
        std::cout << "Trying different goal!" << std::endl;
        found_new = true;
      }
    }
    if(!found_new) return false;
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
    for (const vx_id &vx: start.currentObsLimitedVxs) {
      if (vx == goal.vx) return false;
    }
    return true;
  }
  // Get the approaching angle of own ship wrt target ship
  Eigen::Vector2d path = goal.position - start.position;
  double theta = GetBearing(path, goal.obs_heading);
  // Check colregs depending on situation
  if (abs(theta) <= HeadOnAngle) {
    // head on
    if (goal.vx == FR) {
      // should be FL
      return false;
    }
    goal.currentObsLimitedVxs.push_back(FR);
  }else if(abs(theta) >= OvertakingAngle){
    // overtaking
      goal.overtakingObsList.push_back(goal.obs);
      // avoid future crossings
  }else if(theta > 0){
    // crossing from right, stand on
    if (goal.vx == RR || goal.vx == RL) {
      // should stand on (does it make sense to ignore completely obs that should give way?)
      return false;
    }
  }else if(theta < 0){
    // crossing from left, give way
    if (goal.vx == FR || goal.vx == FL) {
      // should give way
      return false;
    }
    goal.currentObsLimitedVxs.push_back(FR);
    goal.currentObsLimitedVxs.push_back(FL);
  }
  return true;
}

// Check if the path between start and goal collide with any obstacle
bool path_planner::CheckCollision(const Node &start, Node &goal, const std::shared_ptr<std::vector<Node>>& collision_points) {
  Eigen::Vector3d start3d = {start.position.x(), start.position.y(), 0};
  Eigen::Vector3d goal3d = {goal.position.x(), goal.position.y(), 0};
  // vehicle path to check for collision
  Eigen::Vector3d path = goal3d - start3d;
  // time from start to goal
  goal3d[2] = path.norm() / v_info_.speed;
  //path = goal3d - start3d;

  // plot
  plotCKFile_ << "---" << std::endl;
  plotCKFile_ << "Start_" << start3d(0) << "_" << start3d(1) << "_" << start3d(2) << std::endl;
  plotCKFile_ << "Goal_" << goal3d(0) << "_" << goal3d(1) << "_" << goal3d(2) << std::endl;

  for (const obs_ptr& obs_ptr: obss_info_.obstacles) {
    Obstacle obs = *obs_ptr.get();

    if (colregs_compliance) {
      double theta = GetBearing(Eigen::Vector2d(path.x(), path.y()), obs.heading);
      if (theta > HeadOnAngle && theta < OvertakingAngle) {
        // crossing from right, stand on
        continue;
      }
    }

    // Compute obstacle direction in x-y-time
    Eigen::Vector3d bb_direction(obs.speed * cos(obs.heading), obs.speed * sin(obs.heading), 1);
    //bb_direction.normalize(); no, because multiplied by t' it returns the position at time t'

    // Find vertexes in abs frame
    std::vector<Vertex> vxs_abs;
    obs.FindAbsVxs(start.time, vxs_abs);

    // Plot stuff
    {
      plotCKFile_ << "Obs_" << obs.id << std::endl;
      // Uncomment to plot
      //plotCKFile_ << "PlotIt" << std::endl;
      plotCKFile_ << "Direction_" << bb_direction(0) << "_" << bb_direction(1) << "_" << bb_direction(2) << std::endl;
      for (Vertex vx: vxs_abs) {
        plotCKFile_ << "Vx_" << vx.position.x() << "_" << vx.position.y() << std::endl;
      }
      plotCKFile_ << "-" << std::endl;
    }

    // Do not check the goal obstacle: the algorithm already aims for the visible vxs
    //  this condition is here just to print plot stuff before exiting
    if (obs.id == goal.obs) {
      continue;
    }

    // search each of the bb 4 sides for collisions with path
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

      // if the path starts from the current obs, do not check the sides of the departing vx.
      //  still, the diagonals of start.obs will be checked for collision
      if (obs.id == start.obs && (start.vx == vx_idx1 || start.vx == vx_idx2)) {
        continue;
      }
      /*blindly jump the check here can cause collision when the goal is on another bb side that is then not incident with the path
      resulting in false negative outcome and so a collision because the current side is traversed by own ship
      */

      Eigen::Vector3d collision_point;
      if (FindLinePlaneIntersectionPoint(vxs_abs[vx_idx1], vxs_abs[vx_idx2], bb_direction, start3d, goal3d,
                                         collision_point)) {
        // The desired path crosses the face of the bb defined by the two vxs and its direction
        Node cp_node;
        Eigen::Vector2d collision_point_2d(collision_point.x(), collision_point.y());
        cp_node.position = collision_point_2d;
        cp_node.obs = obs.id;
        cp_node.time = start.time + collision_point.z();
        if (collision_points != nullptr) collision_points->push_back(cp_node);
        return false;
      }
    }

    // check the diagonal
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

      Eigen::Vector3d collision_point;
      if (FindLinePlaneIntersectionPoint(vxs_abs[vx_idx1], vxs_abs[vx_idx2], bb_direction, start3d, goal3d,
                                         collision_point)) {
        return false;
      }
    }
  }
  plotCKFile_ << "Good" << std::endl;
  return true;
}

bool path_planner::FindLinePlaneIntersectionPoint(Vertex vx1, Vertex vx2, const Eigen::Vector3d &bb_direction,
                                                  const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
                                                  Eigen::Vector3d &collision_point) const {
  Eigen::Vector3d path = goal - start;
  Eigen::Vector3d vx1_pos(vx1.position.x(), vx1.position.y(), 0);
  Eigen::Vector3d vx2_pos(vx2.position.x(), vx2.position.y(), 0);

  // Get the plane normal
  Eigen::Vector3d side = vx1_pos - vx2_pos;
  Eigen::Vector3d planeNormal = bb_direction.cross(side);
  planeNormal.normalize();

  // if the plane and the path are //
  if (planeNormal.dot(path) == 0) return false; //do we need tolerance here?

  // Compute position along the path where the collision happens (from http://paulbourke.net/geometry/pointlineplane/)
  Eigen::Vector3d startToOne = vx1_pos - start;

  double c_pos = planeNormal.dot(startToOne) / planeNormal.dot(path);
  c_pos = std::round(c_pos * 1000.0) / 1000.0;

  if (c_pos >= 0 && c_pos <= 1) { // intersect_point between start and goal
    // there's the equals since the function does NOT care for the starting vx sides and the goal obs
    // Get collision time and point
    Eigen::Vector3d doable_path = path * c_pos; // Part of the path prior the collision
    doable_path.z() = 0;
    double collision_time = doable_path.norm() / v_info_.speed; // When the collision happens
    doable_path.z() = collision_time;
    collision_point = start + doable_path; // Where the collision happens
    // Get vertexes at time t'
    Eigen::Vector3d vertex1_position = vx1_pos + bb_direction * collision_time;
    Eigen::Vector3d vertex2_position = vx2_pos + bb_direction * collision_time;
    // Check if point is inside those two vertexes
    Eigen::Vector3d P1 = collision_point - vertex1_position;
    Eigen::Vector3d P2 = collision_point - vertex2_position;
    if (P1.dot(P2) <= 0) return true;
  }
  return false;
}

void path_planner::BuildPath(Node goal, std::stack<Node> &waypoints) {    // Build path from goal to initial position
  Node current = std::move(goal);
  while (current.parent != nullptr) {
    Node wp;
    wp.position = current.position;
    wp.time = current.time;
    wp.vx = current.vx;

    waypoints.push(wp);
    current = *(current.parent);
    //  Plot stuff
    {
      plotWpsFile_ << "Time_" << wp.time << std::endl;
      plotWpsFile_ << "Waypoint_" << wp.position(0) << "_" << wp.position(1) << std::endl;
      for (const obs_ptr& obs_ptr: obss_info_.obstacles) {
        Obstacle obs = *obs_ptr.get();
        plotWpsFile_ << "Obs_" << obs.id << std::endl;
        Eigen::Vector2d position = ComputePosition(obs, wp.time);
        plotWpsFile_ << "Position_" << position(0) << "_" << position(1) << std::endl;
        plotWpsFile_ << "Heading_" << obs.heading << std::endl;
        plotWpsFile_ << "Dimx_" << obs.dim_x << std::endl;
        plotWpsFile_ << "Dimy_" << obs.dim_y << std::endl;
        plotWpsFile_ << "Safety_" << obs.safety_bb_ratio << std::endl;
        std::vector<Vertex> vxs_abs;
        obs.FindAbsVxs(wp.time, vxs_abs);
        for (Vertex &vx: vxs_abs) {
          plotWpsFile_ << "Vx_" << vx.position.x() << "_" << vx.position.y() << std::endl;
        }
        //std::cout << vxs_abs[wp.vx].position.x() << " " << vxs_abs[wp.vx].position.y() << " _ "  << std::endl;
        plotWpsFile_ << "-" << std::endl;
      }
    }
  }
  // Plot stuff
  {
    plotWpsFile_ << "Time_" << "0" << std::endl;
    plotWpsFile_ << "Waypoint_" << v_info_.position(0) << "_" << v_info_.position(1) << std::endl;
    for (const obs_ptr& obs_ptr: obss_info_.obstacles) {
      Obstacle obs = *obs_ptr.get();
      plotWpsFile_ << "Obs_" << obs.id << std::endl;
      Eigen::Vector2d position = ComputePosition(obs, 0);
      plotWpsFile_ << "Position_" << position(0) << "_" << position(1) << std::endl;
      plotWpsFile_ << "Heading_" << obs.heading << std::endl;
      plotWpsFile_ << "Dimx_" << obs.dim_x << std::endl;
      plotWpsFile_ << "Dimy_" << obs.dim_y << std::endl;
      plotWpsFile_ << "Safety_" << obs.safety_bb_ratio << std::endl;
      std::vector<Vertex> vxs_abs;
      obs.FindAbsVxs(0, vxs_abs);
      for (Vertex &vx: vxs_abs) {
        plotWpsFile_ << "Vx_" << vx.position.x() << "_" << vx.position.y() << std::endl;
      }
      plotWpsFile_ << "-" << std::endl;
    }
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
      double k = atan2(-vertex_vehicle.y(), vertex_vehicle.x()); // error for (0,0)

      // NORM OF OBS_SPEED HAS TO BE EQUAL OR SMALLER THAN V_SPEED
      // or, more complete, the argument of the asin has to be between -1 and +1
      if (abs(obstacle.speed * sin(obstacle.heading + k) / v_info_.speed) <= 1) {
        double theta = asin(obstacle.speed * sin(obstacle.heading + k) / v_info_.speed) - k;
        double t;
        if (abs(vertex_vehicle.x()) > 0.001) { // should it be something more than 0?
          t = vertex_vehicle.x() / (v_info_.speed * cos(theta) - obstacle.speed * cos(obstacle.heading));
        } else {
          t = vertex_vehicle.y() / (v_info_.speed * sin(theta) - obstacle.speed * sin(obstacle.heading));
        }
        // just bounding the time does it make sense?
        if(t>0 && t < MAX_TIME) {
          Eigen::Vector2d intercept_point_position;
          intercept_point_position.x() = vehicle_position.x() + v_info_.speed * cos(theta) * t;
          intercept_point_position.y() = vehicle_position.y() + v_info_.speed * sin(theta) * t;

          // Check if point is in an obstacle bb => overlapping bbs, ignore the point
          std::shared_ptr<std::vector<obs_ptr>> surrounding_obs(new std::vector<obs_ptr>);
          bool isInAnyBB = IsInAnyBB(intercept_point_position, t, surrounding_obs);
          if (isInAnyBB && surrounding_obs->size() == 1 && surrounding_obs->at(0)->id == obstacle.id) isInAnyBB = false;

          // Save the point
          if (!isInAnyBB) {
            it->ip_position = intercept_point_position;
            it->ip_time = t;
            ++it;
            continue;
          }
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

    std::vector<double> thetas;  // angles wrt abs frame
    std::vector<Eigen::Vector2d> vxs_vh;

    for (Vertex &vx: vxs_abs) {
      Eigen::Vector2d vx_vh = vx.position - node.position; //if 0 there's a problem
      double sign = obs_vh.x() * vx_vh.y() - obs_vh.y() * vx_vh.x(); // same as z value of a cross product (and so its direction)
      double theta = sign / abs(sign) * acos(obs_vh.normalized().dot(vx_vh.normalized()));
      thetas.push_back(theta);
      vxs_vh.push_back(vx_vh);
    }
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
                             const std::shared_ptr<std::vector<obs_ptr>>& surrounding_obs) const {
  for (const obs_ptr& obs_ptr: obss_info_.obstacles) {
    if (IsInBB(element_pos, obs_ptr, time)) {
      if (surrounding_obs != nullptr) {
        surrounding_obs->push_back(obs_ptr);
      }
      return true;
    }
  }
  return false;
}

bool path_planner::IsInBB(const Eigen::Vector2d &element_pos, const obs_ptr& obs, double time) {

  Eigen::Vector2d bodyObs_element = GetProjectionInObsFrame(element_pos, *obs, time);

  // asymmetric bb x dimension computation
  double theta = atan2(bodyObs_element.y(), bodyObs_element.x()); // Approaching angle, error for (0,0)
  // choose comparing dimension based on theta
  bool IsAhead = (abs(theta) <= M_PI / 2);
  if (IsAhead) {
    return (abs(bodyObs_element.x()) < abs(obs->vxs[0].position.x()) &&
            abs(bodyObs_element.y()) < abs(obs->vxs[0].position.y()));
  } else {
    return (abs(bodyObs_element.x()) < abs(obs->vxs[2].position.x()) &&
            abs(bodyObs_element.y()) < abs(obs->vxs[2].position.y()));
  }
}

void path_planner::FindExitVxs(const Eigen::Vector2d &element_pos, const obs_ptr& obs, double time,
                               std::vector<vx_id> &allowedVxs) {
  Eigen::Vector2d bodyObs_e = GetProjectionInObsFrame(element_pos, *obs, time);

  bool IsLeftOfDiag1 = (bodyObs_e.y() >=
                obs->vxs[1].position.y() / obs->vxs[1].position.x() * bodyObs_e.x()); // Being left of diag FL-RR
  bool IsLeftOfDiag2 = (bodyObs_e.y() >=
                obs->vxs[0].position.y() / obs->vxs[0].position.x() * bodyObs_e.x()); // Being left of diag FR-LL

  if (IsLeftOfDiag1 && IsLeftOfDiag2) {
    // USV in port side of bb
    //  go for FL or RL
    allowedVxs.push_back(FL);
    allowedVxs.push_back(RL);
  } else {
    if (IsLeftOfDiag1) {
      // USV in stern side of bb
      //  go for RL or RR
      allowedVxs.push_back(RL);
      allowedVxs.push_back(RR);
    } else {
      if (IsLeftOfDiag2) {
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

bool path_planner::CheckPath(const Eigen::Vector2d &vh_pos, double time, std::stack<Node> &waypoints) {
  // the waypoint should be just the ones left to reach, not the whole path returned by the library
  if(waypoints.empty()){
    return false;
  }

  Node start;
  start.position = vh_pos;
  start.time = time;

  while (!waypoints.empty()) {
    Node goal = waypoints.top();
    waypoints.pop();
    //std::cout << "Pos: " << wp.position << std::endl;
    //std::cout << "Time: " << wp.time << std::endl;

    if(!CheckCollision(start, goal)) return false;

    start = goal; // check =operator definition
  }
  return true;
}


