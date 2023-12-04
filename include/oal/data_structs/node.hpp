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

struct Metrics{
    double n_ancestors = 0;
    double maxHeadingChange = 0;  // this should not start from zero if initial OS heading is known
    double totHeadingChange = 0;  // this should not start from zero if initial OS heading is known
    double averageSpeed = 0;
    double totDistance = 0;
    double maxSpeed = 0;
    double maxSpeedChange = 0;
};

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

    bool hasSimilar = false;

    Metrics mtrcs;


    Node() = default;

    Node(const TPoint& point, const std::shared_ptr<Obstacle> &obs_ptr, vx_id vx, Node& parent_node)
            : obs_ptr(obs_ptr), vx(vx) {
      position = point.pos;
      time = parent_node.time +  point.time;
      SetParent(parent_node);
    }

    Node(const Node& other) {
      // Copy all members from 'other' to 'this'
      position = other.position;
      time = other.time;
      costToReach = other.costToReach;
      costToGoal = other.costToGoal;
      costTotal = other.costTotal;
      vh_speed = other.vh_speed;
      obs_ptr = other.obs_ptr;
      vx = other.vx;
      parent = other.parent;  // assuming that shared_ptr copy is what you want
      currentObsLimitedVxs = other.currentObsLimitedVxs;
      overtakingObsList = other.overtakingObsList;
      mtrcs = other.mtrcs;
      /*n_ancestors = other.n_ancestors;
      maxHeadingChange = other.maxHeadingChange;
      totHeadingChange = other.totHeadingChange;
      averageSpeed = other.averageSpeed;
      totDistance = other.totDistance;
      maxSpeed = other.maxSpeed;
      maxSpeedChange = other.maxSpeedChange;*/
    }

    // Used to order nodes in set according to total cost to reach the goal
    bool operator<(const Node &other) const {
      return costTotal < other.costTotal;
    }

    // Nodes are equal if position, time, speed, obs and vx are the same (small epsilon involved for position and time)
    bool operator==(const Node &other) const {
      return (position-other.position).norm()<0.01 &&                       // same position
             abs(time-other.time)<0.5 &&                                 // same time
             obs_ptr.get() == other.obs_ptr.get() && vx == other.vx &&      // same obs and vx
             parent->obs_ptr.get() == other.parent->obs_ptr.get() &&        // same parent obs
             parent->vx == other.parent->vx;                                 // same parent vx
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

    // If newly created node already exists in set and second heuristic says it's better, erase the already existing one
    bool RemoveWorstDuplicates(std::multiset<Node> &set) const;

    bool HasAncestor(const Node &node) const;

    // Set node parent, inherit its overtakingObsList and update "alternative costs"
    void SetParent(const Node& parent_node) {
      parent = std::make_shared<Node>(parent_node);
      overtakingObsList = parent_node.overtakingObsList;
      // Update mtrcs
      double headingChange = GetHeadingChange(); // [0, pi]
      double speedChange = vh_speed - parent->vh_speed;
      double dist = (position - parent->position).norm();
      // Alternative costs for when two nodes are the same (A* heuristic)
      mtrcs.n_ancestors = parent->mtrcs.n_ancestors + 1;
      mtrcs.totHeadingChange = parent->mtrcs.totHeadingChange + headingChange;
      mtrcs.maxHeadingChange = std::max(parent->mtrcs.maxHeadingChange, headingChange);
      mtrcs.maxSpeed = std::max(parent->mtrcs.maxSpeed, parent->vh_speed);
      mtrcs.maxSpeedChange = std::max(parent->mtrcs.maxSpeedChange, abs(speedChange));  //TODO speedChange is also decelleration
      mtrcs.totDistance = parent->mtrcs.totDistance + dist;
      mtrcs.averageSpeed = (parent->mtrcs.averageSpeed * parent->mtrcs.totDistance + parent->vh_speed * dist) / mtrcs.totDistance;
    }


    double GetHeadingChange() {
      if(parent->parent != nullptr){
        Eigen::Vector2d t1 = position - parent->position;
        Eigen::Vector2d t2 = parent->position - parent->parent->position;
        return std::acos(t1.normalized().dot(t2.normalized())); // [0, pi]
      }
      return 0; //TODO if starting heading is known, you can actually compute the heading change
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


    bool RemoveWorstDuplicates(std::multiset<Node> &set);
};


#endif //OAL_NODE_HPP
