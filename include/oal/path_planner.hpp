#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

#include <iostream>
#include <fstream>
#include <memory>
#include <stack>
#include <set>
#include <eigen3/Eigen/Eigen>
#include "oal/data_structs/misc.hpp"
#include "oal/helper_functions.hpp"

#define MAX_TIME 10000
#define HeadOnAngle (15*(M_PI/180))
#define OvertakingAngle (112*(M_PI/180))

class path_planner {
private:
    VehicleInfo v_info_;
    ObstaclesInfo obss_info_;
    bool colregs_compliance;
    std::vector<std::string> noCrossList;

    std::ofstream plotWpsFile_;
    std::ofstream plotCKFile_;

    // Given some obstacle vertexes, find (if they exist) the points where they are intercepted by own ship
    void FindInterceptPoints(const Node &start, Obstacle &obstacle,
                             std::vector<Vertex> &vxs_abs);

    // Given a bb side (2 vxs), finds where a vector from start to goal collides with the plane generated by the bb side and direction
    static bool FindLinePlaneIntersectionPoint(Vertex vx1, Vertex vx2, const Eigen::Vector3d &bb_direction,
                                               const Node &start, const Eigen::Vector3d &goal,
                                               Eigen::Vector3d &collision_point);

    // Check if a point is in any obs bb at given time (and save such obs in vector if its pointer is given)
    bool IsInAnyBB(TPoint time_point, const std::shared_ptr<std::vector<obs_ptr>> &surrounding_obs = nullptr);

    // Check if starting position is safe or find new start positions. Insert them in set
    bool RootSetup(const Eigen::Vector2d &goal_position, std::multiset<Node> &open_set);

    // Check if path between nodes is colregs compliant
    bool CheckColreg(const Node &start, Node &goal) const;

    // Check if path between nodes collide with any obstacle (and save such points in vector if its pointer is given)
    bool
    CheckCollision(const Node &start, Node &goal, const std::shared_ptr<std::vector<Node>> &collision_points = nullptr);

    // Check the final path to goal. If goal is unreachable only because it is in an obs bb, finds new goal outside it
    bool CheckFinal(const Node &start, Node &goal);

    // Order waypoints in a stack by going backward from the goal to start using the parent pointer attribute
    void BuildPath(const Node &goal, Path &path);

    void FindObssLocalVxs(bool with_uncertainty);

public:
    // vehicle start position and obstacles information are supposed to be taken in the same time instant.
    path_planner() = default;

    path_planner(VehicleInfo v_info, const std::vector<Obstacle>& obstacles)
            : v_info_(std::move(v_info)) {
      colregs_compliance = false;
      for(const auto& obs : obstacles){
        obss_info_.obstacles.push_back(std::make_shared<Obstacle>(obs));
      }

      // Plot stuff
      {
        if (plotCKFile_.is_open()) plotCKFile_.close();
        plotCKFile_.open("CKlog.txt", std::ofstream::trunc);
      }
    }

    // Compute the path to reach the goal and fills the path's waypoints stack
    bool ComputePath(const Eigen::Vector2d &goal, bool colregs, Path &path);

    // Given a trajectory, checks if it's safe
    bool CheckPath(const Eigen::Vector2d &vh_pos, Path path);

    void SetVhData(VehicleInfo v_info) {
      v_info_ = std::move(v_info);
    }

    void SetObssData(const std::vector<Obstacle>& obstacles) {
      obss_info_.obstacles.clear();
      for(const auto& obs : obstacles){
        obss_info_.obstacles.push_back(std::make_shared<Obstacle>(obs));
      }
    }
};

#endif
