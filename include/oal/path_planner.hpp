#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

#include <iostream>
#include <fstream>
#include <memory>
#include "data_structs.hpp"
#include "helper_functions.hpp"
#include <stack>
#include <set>
#include <eigen3/Eigen/Eigen>


class path_planner {
private:
    VehicleInfo v_info_;
    ObstaclesInfo obss_info_;
    std::ofstream plotWpsFile_;
    std::ofstream plotCKFile_;


    // Given some obstacle vertexes, find the intercept points with the vehicle
    void FindInterceptPoints(const Eigen::Vector2d &vehicle_position, Obstacle &obstacle,
                                std::vector<Vertex> &vxs_abs) const;

    static bool CheckColreg(const Node &start, Node &goal);

    // Check if the path between start and goal collide with any obstacle
    bool CheckCollision(Node start, Node &goal, bool isFinalGoal);

    // Given a set of vertexes, find if are visible from the vehicle (ignoring other obstacles)
    static void FindVisibility(Node &node, Obstacle &obs, std::vector<Vertex> &vxs_abs);

    void BuildPath(Node *it, std::stack<Node> &waypoints);

    static bool FindLinePlaneIntersectionPoint(Vertex vx1, Vertex vx2, const Eigen::Vector3d& bb_direction, const Eigen::Vector3d& start, const Eigen::Vector3d& goal, double speed, Eigen::Vector3d &collision_point);

    static bool IsInBB(const Eigen::Vector2d& element_pos, Obstacle &obs, double time);

    static void FindExitVxs(const Eigen::Vector2d& element_pos, Obstacle &obs, double time, std::vector<vx_id> &allowedVxs);


public:
    // vehicle start position and obstacles information are supposed to be taken in the same time instant.
    // the user can always compute the update position/obstacles info before creating the class

    path_planner(VehicleInfo v_info, ObstaclesInfo obss_info)
            : v_info_(std::move(v_info)), obss_info_(std::move(obss_info)) {
      // Plot stuff
      if (plotCKFile_.is_open()) {
        plotCKFile_.close();
      }
      plotCKFile_.open("CKlog.txt", std::ofstream::trunc);

      if (plotWpsFile_.is_open()) {
        plotWpsFile_.close();
      }
      plotWpsFile_.open("WPlog.txt", std::ofstream::trunc);
    }

    // Compute the path to reach the goal and fills the waypoints stack
    bool ComputePath(const Eigen::Vector2d &goal, std::stack<Node> &waypoints);

};

#endif
