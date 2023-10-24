#ifndef OAL_NODE_HPP
#define OAL_NODE_HPP

#include <eigen3/Eigen/Eigen>
#include <utility>
#include <memory>
#include <iostream> //debug
#include <set>
#include <iomanip>
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
    double vh_speed = 0;
    std::shared_ptr<Obstacle> obs_ptr = nullptr;
    vx_id vx = NA;
    std::shared_ptr<Node> parent = nullptr;
    std::vector<vx_id> currentObsLimitedVxs;
    std::vector<std::string> overtakingObsList;

    Node() = default;

    Node(Eigen::Vector2d pos, const std::shared_ptr<Obstacle> &obs_ptr, vx_id vx, double time)
            : position(std::move(pos)), obs_ptr(obs_ptr), vx(vx), time(time) {}
    //obs(obs_ptr->id), obs_heading(obs_ptr->heading)

    /*Node(const Node &other){
      position = other.position;
      time = other.time;
      vx = other.vx;
      if(other.parent != nullptr) vh_speed = other.parent->vh_speed;
    }*/

    // Used to order nodes in set according to total cost to reach the goal
    bool operator<(const Node &other) const {
      return costTotal < other.costTotal;
    }

    bool operator==(const Node &other) const {
      Eigen::Vector2d pos_rounded(std::round(position[0] * 100) / 100, std::round(position[1] * 100) / 100);
      Eigen::Vector2d other_pos_rounded(std::round(other.position[0] * 100) / 100,
                                        std::round(other.position[1] * 100) / 100);
      return other_pos_rounded == pos_rounded && obs_ptr.get() == other.obs_ptr.get() && vx == other.vx &&
             time == other.time && vh_speed == other.vh_speed;
    }

    // Set estimated cost and total cost according to own ship speed
    void UpdateCosts(const Eigen::Vector2d &goal);

    // Given some nodes, get the closer
    void GetCloser(const std::shared_ptr<std::vector<Node>> &nodes_list, Node &closer) const;

    bool IsUnique(Node other);

    void FindVisibilityVxs(Obstacle target_obs, std::vector<Vertex> &vxs_abs);

    bool IsInSet(std::multiset<Node> &set);

    void print() const {
      std::cout << std::setprecision(3);
      Node node = *this;
      std::cout << "NODE: " << std::endl;
      while (node.parent != nullptr) {
        std::cout << " Time: " << node.time << "  Pos: " << node.position.x() << " " << node.position.y();
        std::cout << " Obs: " << node.obs_ptr->id << "/" << (vx_id) node.vx << "  speed: " << node.vh_speed
                  << std::endl;
//        std::cout<<std::endl;
        node = *node.parent;
      }
    }

};


#endif //OAL_NODE_HPP
