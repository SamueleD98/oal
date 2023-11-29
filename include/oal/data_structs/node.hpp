#ifndef OAL_NODE_HPP
#define OAL_NODE_HPP

#include <eigen3/Eigen/Eigen>
#include <utility>
#include <memory>
#include <iostream> //debug
#include <set>
#include <iomanip>
#include "oal/data_structs/vertex.hpp"
#include "oal/data_structs/obstacle.hpp"

class Node {
public:
    Eigen::Vector2d position; //vehicle position
    // time and costToReach are the same when time to reach the target is the cost function
    double time = -1;  // time instant
    double costToReach = -1; //cost to reach the Node
    double costToGoal = -1; //estimated cost to reach Goal
    double costTotal = -1; //total cost
    double vh_speed = 0; //vehicle speed from this node to next
    std::shared_ptr<Obstacle> obs_ptr = nullptr;
    vx_id vx = NA;
    std::shared_ptr<Node> parent = nullptr;
    std::vector<vx_id> currentObsLimitedVxs; //not viable vxs depending on maneuver
    std::vector<std::string> overtakingObsList; //list of obs to overtake from origin up to node

    Node() = default;

    Node(const TPoint& point, const std::shared_ptr<Obstacle> &obs_ptr, vx_id vx, const Node& parent_node)
            : obs_ptr(obs_ptr), vx(vx) {
      position = point.pos;
      time = parent_node.time +  point.time;
      SetParent(parent_node);
    }

    // Used to order nodes in set according to total cost to reach the goal
    bool operator<(const Node &other) const {
      return costTotal < other.costTotal;
    }

    // Nodes are equal if position, time, speed, obs and vx are the same (small epsilon involved for position and time)
    bool operator==(const Node &other) const {
      return (position-other.position).norm()<0.01 && obs_ptr.get() == other.obs_ptr.get() && vx == other.vx &&
             time-other.time<0.5; //&& vh_speed == other.vh_speed;
    }

    // Set estimated cost and total cost according to own ship speed
    void UpdateCosts(const Eigen::Vector2d &goal);

    // Given some nodes, get the closer one
    void GetCloser(const std::shared_ptr<std::vector<Node>> &nodes_list, Node &closer) const;

    // Check if its obs and vx have already been visited in its chain of nodes
    bool IsUnique(Node other);

    // Given a set of vertexes, find which are visible from the vehicle when in this state (node)
    void FindVisibilityVxs(Obstacle target_obs, std::vector<Vertex> &vxs_abs);

    // Find the exit vxs that wouldn't need to cross the main diagonals and save them in a given vector
    void FindExitVxs(const Obstacle &obs, std::vector<vx_id> &allowedVxs) const;

    // Check if newly created node already exists in set
    // TODO if it does, maybe it has better properties (less change in direction, smaller max change, less speed change, smaller average speed)
    bool IsInSet(std::multiset<Node> &set) const;

    // Set node parent and inherit its overtakingObsList
    void SetParent(const Node& parent_node){
      parent = std::make_shared<Node>(parent_node);
      overtakingObsList = parent_node.overtakingObsList;
    }

    void print() const {
      std::cout << std::setprecision(3);
      Node node = *this;
      if(node.parent != nullptr) std::cout <<std::endl << "  trace: "<<std::endl;
      while (node.parent != nullptr) {
        std::cout << "   - time: " << node.time << "  Pos: " << node.position.x() << " " << node.position.y();
        if(node.obs_ptr != nullptr){
          std::cout << "   Obs: " << node.obs_ptr->id << "/" << (vx_id) node.vx << "  speed: " << node.vh_speed;
        }
        std::cout << std::endl;
        node = *node.parent;
      }
    }


};


#endif //OAL_NODE_HPP
