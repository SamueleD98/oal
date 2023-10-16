#ifndef OAL_NODE_HPP
#define OAL_NODE_HPP

#include <eigen3/Eigen/Eigen>
#include <utility>
#include <memory>
#include <iostream> //debug
#include <set>
#include "vertex.hpp"
#include "obstacle.hpp"

class Node {
public:
    Eigen::Vector2d position; //vehicle position
    // time and costToReach are the same when time to reach the target is the cost function
    double time = -1;  // time instant
    double costToReach = -1; //cost to reach the Node
    double costToGoal = -1; //estimated cost to reach Goal
    double costTotal = -1; //total cost
    std::string obs;
    double obs_heading = -4;
    vx_id vx;
    std::shared_ptr<Node> parent = nullptr;
    std::vector<vx_id> currentObsLimitedVxs;
    std::vector<std::string> overtakingObsList;

    Node()= default;

    Node(Eigen::Vector2d pos, const std::shared_ptr<Obstacle>& obs_ptr, vx_id vx, double time)
      :position(std::move(pos)), obs(obs_ptr->id), obs_heading(obs_ptr->heading), vx(vx), time(time){}

    // Used to order nodes in set according to total cost to reach the goal
    bool operator<(const Node &other) const {
      return costTotal < other.costTotal;
    }

    // Set estimated cost and total cost according to own ship speed
    void UpdateCosts(const Eigen::Vector2d &goal, double vh_speed);

    // Given some nodes, get the closer
    void GetCloser(const std::shared_ptr<std::vector<Node>>& nodes_list, Node& closer ) const;

    bool IsIn(std::multiset<Node> &set);
};


#endif //OAL_NODE_HPP
