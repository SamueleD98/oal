#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

#include <iostream>
#include <fstream>
#include <memory>
#include <stack>
#include <set>
#include <eigen3/Eigen/Eigen>
#include "data_structs/misc.hpp"
#include "helper_functions.hpp"

#define MAX_TIME 100
#define HeadOnAngle (15*(M_PI/180))
#define OvertakingAngle (112*(M_PI/180))

typedef std::shared_ptr<Obstacle> obs_ptr;

class path_planner {
private:
    VehicleInfo v_info_;
    ObstaclesInfo obss_info_;
    bool colregs_compliance;
    std::vector<std::string> noCrossList;

    std::ofstream plotWpsFile_;
    std::ofstream plotCKFile_;

    // Given some obstacle vertexes, find the intercept points with the vehicle
    void FindInterceptPoints(const Eigen::Vector2d &vehicle_position, Obstacle &obstacle,
                             std::vector<Vertex> &vxs_abs) const;

    // Check if path between nodes is colregs compliant
    bool CheckColreg(const Node &start, Node &goal) const;

    // Check if the path between nodes collide with any obstacle (and save such points in vector if its pointer is given)
     bool CheckCollision(const Node &start, Node &goal, const std::shared_ptr<std::vector<Node>>& collision_points = nullptr);

    // Given a set of vertexes, find if are visible from the vehicle (ignoring other obstacles)
    static void FindVisibility(Node &node, Obstacle &obs, std::vector<Vertex> &vxs_abs);

    // Order waypoints in a stack by going backward from the goal to start using the parent pointer attribute
    void BuildPath(Node goal, std::stack<Node> &waypoints);

    bool FindLinePlaneIntersectionPoint(Vertex vx1, Vertex vx2, const Eigen::Vector3d &bb_direction,
                                               const Eigen::Vector3d &start, const Eigen::Vector3d &goal, Eigen::Vector3d &collision_point) const;

    // Check if a point is in the obs bb at given time
    static bool IsInBB(const Eigen::Vector2d &element_pos, const obs_ptr& obs, double time);

    // Check if a point is in any obs bb at given time (and save such obs in vector if its pointer is given)
    bool IsInAnyBB(const Eigen::Vector2d &element_pos, double time,
                   const std::shared_ptr<std::vector<obs_ptr>>& surrounding_obs = nullptr) const;

    // Given a point in an obs bb, find the exit vxs that wouldn't need to cross the main diagonals and save them in a given vector
    static void FindExitVxs(const Eigen::Vector2d &element_pos, const obs_ptr& obs, double time, std::vector<vx_id> &allowedVxs);

    // Check the final path to goal. If goal is in an obs bb, finds new goal outside it
    bool CheckFinal(const Node &start, Node &goal);

public:
    // vehicle start position and obstacles information are supposed to be taken in the same time instant.
    // the user can always compute the update position/obstacles info before creating the class

    path_planner(VehicleInfo v_info, ObstaclesInfo obss_info)
            : v_info_(std::move(v_info)), obss_info_(std::move(obss_info)) {
      colregs_compliance = false;
      // Plot stuff
      {
        if (plotCKFile_.is_open()) {
          plotCKFile_.close();
        }
        plotCKFile_.open("CKlog.txt", std::ofstream::trunc);

        if (plotWpsFile_.is_open()) {
          plotWpsFile_.close();
        }
        plotWpsFile_.open("WPlog.txt", std::ofstream::trunc);
      }
    }

    // Compute the path to reach the goal and fills the waypoints stack
    bool ComputePath(const Eigen::Vector2d &goal, bool colregs, Path path);

    // At the moment does not check colregs too, just collisions
    bool CheckPath(const Eigen::Vector2d &vh_pos, double time, std::stack<Node> &waypoints);

    void SetVhData(VehicleInfo v_info){
      v_info_ = std::move(v_info);
    }

    void SetObssData(ObstaclesInfo obss_info){
      obss_info_ = std::move(obss_info);
    }
};

#endif
