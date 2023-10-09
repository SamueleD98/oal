#ifndef OAL_VERTEX_HPP
#define OAL_VERTEX_HPP

// Vertex indexes map
enum vx_id {
    FR = 0, // forward right
    FL = 1, // forward left
    RR = 2, // rear right
    RL = 3  // rear left
};

struct Vertex {
    vx_id id;
    Eigen::Vector2d position;
    bool isVisible = false;
    Eigen::Vector2d ip_position;  //intercept point position
    double ip_time = -1; //intercept time
    //std::shared_ptr<Obstacle> obs = nullptr;
};


#endif //OAL_VERTEX_HPP