#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/State.h>

typedef ompl::base::SE2StateSpace::StateType State;

#include "hybrid_astar_test/algorithm/path_searcher/hybrid_a_star/node3d.h"
#include "hybrid_astar_test/algorithm/path_searcher/hybrid_a_star/node2d.h"
#include "hybrid_astar_test/algorithm/path_searcher/hybrid_a_star/visualize.h"
#include "hybrid_astar_test/algorithm/path_searcher/hybrid_a_star/collision_detection.h"

namespace HybridAStar {
class Node3D;
class Node2D;
class Visualize;

class Algorithm {
public:
    Algorithm() {}
    static Node3D* hybridAStar(Node3D& start,
                               const Node3D& goal,
                               Node3D* nodes3D,
                               Node2D* nodes2D,
                               int width,
                               int height,
                               CollisionDetection& configurationSpace,
                               float* dubinsLookup,
                               Visualize& visualization);

};
}
#endif // ALGORITHM_H
