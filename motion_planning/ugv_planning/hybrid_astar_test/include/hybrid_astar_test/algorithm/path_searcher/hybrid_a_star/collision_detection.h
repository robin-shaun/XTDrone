#ifndef COLLISIONDETECTION_H
#define COLLISIONDETECTION_H

#include <nav_msgs/OccupancyGrid.h>

#include "hybrid_astar_test/global_definition/constants.hpp"
#include "hybrid_astar_test/algorithm/path_searcher/hybrid_a_star/node2d.h"
#include "hybrid_astar_test/algorithm/path_searcher/hybrid_a_star/node3d.h"
#include "hybrid_astar_test/algorithm/path_searcher/hybrid_a_star/lookup.h"

namespace HybridAStar {
namespace {
inline void getConfiguration(const Node2D* node, float& x, float& y, float& t) {
    x = node->getX();
    y = node->getY();
    // avoid 2D collision checking
    t = 99;
}

inline void getConfiguration(const Node3D* node, float& x, float& y, float& t) {
    x = node->getX();
    y = node->getY();
    t = node->getT();
}
}
/*!
   \brief The CollisionDetection class determines whether a given configuration q of the robot will result in a collision with the environment.

   It is supposed to return a boolean value that returns true for collisions and false in the case of a safe node.
*/
class CollisionDetection {
public:
    CollisionDetection();
    template<typename T> bool isTraversable(const T* node) const {
        float cost = 0;
        float x;
        float y;
        float t;
        // assign values to the configuration
        getConfiguration(node, x, y, t);

        // 2D collision test
        if (t == 99) {
            return !grid->data[node->getIdx()];
        }

        if (true) {
            cost = configurationTest(x, y, t) ? 0 : 1;
        } else {
            std::cout << "isTraversable 23" << std::endl;
            cost = configurationCost(x, y, t);
        }

        return cost <= 0;
    }

    float configurationCost(float x, float y, float t) const {return 0;}

    bool configurationTest(float x, float y, float t) const;

    void updateGrid(nav_msgs::OccupancyGrid::Ptr map) {grid = map;}

private:
    nav_msgs::OccupancyGrid::Ptr grid;
    Constants::config collisionLookup[Constants::headings * Constants::positions];
};
}
#endif // COLLISIONDETECTION_H
