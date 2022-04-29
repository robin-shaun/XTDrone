#ifndef HYBRID_ASTAR_TEST_NAVIGATION_PATH_SEARCH_HPP
#define HYBRID_ASTAR_TEST_NAVIGATION_PATH_SEARCH_HPP

#include <iostream>
#include <ctime>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "hybrid_astar_test/global_definition/constants.hpp"
#include "hybrid_astar_test/utility/helper.hpp"
#include "hybrid_astar_test/subscriber/costmap_subscriber.hpp"
#include "hybrid_astar_test/subscriber/goal_subscriber.hpp"
#include "hybrid_astar_test/subscriber/start_subscriber.hpp"
#include "hybrid_astar_test/subscriber/est_start_subscriber.hpp"
#include "hybrid_astar_test/publisher/path_publisher.hpp"
#include "hybrid_astar_test/publisher/tf_broadcaster.hpp"
#include "hybrid_astar_test/publisher/points_publisher.hpp"
#include "hybrid_astar_test/algorithm/path_searcher/hybrid_a_star/collision_detection.h"
#include "hybrid_astar_test/algorithm/path_searcher/hybrid_a_star/node3d.h"
#include "hybrid_astar_test/algorithm/path_searcher/hybrid_a_star/node2d.h"
#include "hybrid_astar_test/algorithm/path_searcher/hybrid_a_star/visualize.h"
#include "hybrid_astar_test/algorithm/path_searcher/hybrid_a_star/algorithm.h"
#include "hybrid_astar_test/algorithm/path_searcher/hybrid_a_star/path.h"
#include "hybrid_astar_test/algorithm/path_searcher/hybrid_a_star/smoother.h"
#include "hybrid_astar_test/algorithm/path_searcher/hybrid_a_star/dynamic_voronoi.h"

#include <memory>
#include <mutex>

namespace HybridAStar {
class HybridAStarGraphSearcher{
public:
    HybridAStarGraphSearcher(ros::NodeHandle& nh);
    ~HybridAStarGraphSearcher();
    bool Run();

private:
    bool Plan(const Eigen::Vector3f& start_pt, const Eigen::Vector3f& goal_pt);
    void GetPath(const Node3D* node, int i = 0, std::vector<Node3D> path = std::vector<Node3D>());
    void DealPath(const std::vector<Node3D>& path_, std::vector<Eigen::Vector3f>& finish_path);
    void PublishPath();


private:
    std::shared_ptr<TfBroadcaster>       tf_broadcaster_ptr_;
    std::shared_ptr<CostmapSubscriber>   costmap_subscriber_ptr_;
    std::shared_ptr<StartSubscriber>     start_subscriber_ptr_;
    std::shared_ptr<EstStartSubscriber>  est_start_subscriber_ptr_;
    std::shared_ptr<GoalSubscriber>      goal_subscriber_ptr_;
    std::shared_ptr<Path>                path_publisher_ptr_;
    std::shared_ptr<PathPublisher>       front_end_path_publisher_ptr_;
    std::shared_ptr<PointsPublisher>     start_point_publisher_ptr_;
    std::shared_ptr<PointsPublisher>     goal_point_publisher_ptr_;
    std::shared_ptr<Smoother>            smoother_ptr_;
    DynamicVoronoi                       voronoiDiagram;
    Path                                 smoothedPath = Path(true);

private:
    float   x_origin_;
    float   y_origin_;
    bool    get_map = false;
    bool    re_plan_flag = false;
    bool    arrived_flag = false;
    std::vector<Node3D> path;
    std::vector<Node3D> smooth_path;
    std::vector<Eigen::Vector3f> vis_path;
    std::vector<Eigen::Vector3f> vis_smooth_path;

    float* dubinsLookup = new float [Constants::headings * Constants::headings * Constants::dubinsWidth * Constants::dubinsWidth];
    Constants::config collisionLookup[Constants::headings * Constants::positions];
    Visualize visualization;

private:
    ros::NodeHandle nh_;
    nav_msgs::OccupancyGrid::Ptr grid;
    nav_msgs::OccupancyGrid::Ptr grid_;
    CollisionDetection configurationSpace;

    Eigen::Vector3f goal_point_;

    std::mutex mtx_;
};

}

#endif