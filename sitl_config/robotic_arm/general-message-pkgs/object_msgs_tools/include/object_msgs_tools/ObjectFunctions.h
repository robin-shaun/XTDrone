#ifndef OBJECT_MSGS_TOOLS_OBJECTFUNCTIONS_H
#define OBJECT_MSGS_TOOLS_OBJECTFUNCTIONS_H

#include <object_msgs/Object.h>
#include <geometry_msgs/PoseStamped.h>

namespace object_msgs_tools {

/**
 * Provides a collection of convenience function for
 * object_msgs/Object.
 *
 * \author Jennifer Buehler
 * \date March 2016
 */
class ObjectFunctions
{
public:

     ObjectFunctions(){}

    /**
     * Get the object pose out of the information in \e object according to what's specified
     * in the message (fields object.primitive_origin and/or object.mesh_origin).
     * If both mesh and primitive origins are enabled, the primitives are used to determine
     * the object pose.
     * \return false if insufficient information given to determine object pose, true if
     *      pose is returned in \e pose.
     */
    static bool getObjectPose(const object_msgs::Object& object, geometry_msgs::PoseStamped& pose); 

private:

    /**
     * \param idx the value of either Object::primitive_origin or Object::mesh_origin
     * \param object_header the header of the Object message.
     * \param poses the poses, either Object::primitive_poses or Object::mesh_poses
     * \param origin the field Object::origin, in case it is required.
     */
    static bool getPoseFromFields(const std_msgs::Header& object_header, int idx,
        const std::vector<geometry_msgs::Pose>& poses,
        const geometry_msgs::Pose & origin,
        geometry_msgs::PoseStamped& pose);

    /**
     *  Computes the average pose of several positions.
     *  Because the average of several quaternions
     *  can only be determined approximately when
     *  the quaternions are close together, the orientation
     *  parts are ignored, and only the position is returned.
     */
    static geometry_msgs::Point getAveragePointFrom(const std::vector<geometry_msgs::Pose>& poses); 
};

}

#endif  // OBJECT_MSGS_TOOLS_OBJECTFUNCTIONS_H
