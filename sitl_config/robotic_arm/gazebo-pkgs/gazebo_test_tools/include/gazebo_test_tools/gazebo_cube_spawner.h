#ifndef GAZEBO_TEST_TOOLS_GAZEBOCUBESPAWNER
#define GAZEBO_TEST_TOOLS_GAZEBOCUBESPAWNER

#include <ros/ros.h>

namespace gazebo_test_tools {

/**
 * Spawns a cube of given size, position and orientation into
 * Gazebo.
 *
 * \author Jennifer Buehler
 */
class GazeboCubeSpawner {
public:
    GazeboCubeSpawner(ros::NodeHandle &n);
    
    void spawnCube(const std::string& name, const std::string& frame_id,
            float x, float y, float z, float qx, float qy, float qz, float qw,
            float w=0.05, float h=0.05, float d=0.05, float mass=0.05); 
    
    /**
     * \param isCube if true, spawn a cube. If false, spawn cylinder,
     *      where \e w is the radius and \e h is the height (\e d will be ignored).
     */
    void spawnPrimitive(const std::string& name, const bool isCube,
            const std::string& frame_id,
            float x, float y, float z, float qx, float qy, float qz, float qw,
            float w=0.05, float h=0.05, float d=0.05, float mass=0.05); 

private:

    ros::NodeHandle nh;
    ros::ServiceClient spawn_object;

};

}  // namespace 

#endif  // GAZEBO_TEST_TOOLS_GAZEBOCUBESPAWNER
