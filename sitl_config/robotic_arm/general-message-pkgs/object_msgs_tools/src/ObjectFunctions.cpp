#include <ros/ros.h>
#include <object_msgs_tools/ObjectFunctions.h>
#include <eigen_conversions/eigen_msg.h>

using object_msgs_tools::ObjectFunctions;

bool ObjectFunctions::getObjectPose(const object_msgs::Object& object, geometry_msgs::PoseStamped& pose)
{
    if ((object.primitive_origin == object_msgs::Object::ORIGIN_UNDEFINED) &&
        (object.mesh_origin == object_msgs::Object::ORIGIN_UNDEFINED))
    {
        return false;
    }
    
    bool success = false;
    if (object.primitive_origin != object_msgs::Object::ORIGIN_UNDEFINED)
    {
        success = getPoseFromFields(object.header,object.primitive_origin,
                                          object.primitive_poses, object.origin, pose);
    }
    else  // the mesh origin must be defined because of check above
    {
        success = getPoseFromFields(object.header,object.mesh_origin,
                                          object.mesh_poses, object.origin, pose);
    }
    return success;
}

bool ObjectFunctions::getPoseFromFields(const std_msgs::Header& object_header, int idx,
    const std::vector<geometry_msgs::Pose>& poses,
    const geometry_msgs::Pose & origin,
    geometry_msgs::PoseStamped& pose)
{
    if (idx == object_msgs::Object::ORIGIN_UNDEFINED)
        return false;

    if (idx == object_msgs::Object::ORIGIN_AVERAGE)
    {   
        pose.pose.position = getAveragePointFrom(poses);
        pose.pose.orientation = origin.orientation;
    }
    else if (idx == object_msgs::Object::ORIGIN_CUSTOM)
    {
        pose.pose = origin;
    }
    else if (idx > 0)
    {
        if (poses.size() <= idx)
        {
            ROS_ERROR_STREAM("ObjectFunctions: Inconsistent object, "<<
                "has less primitive poses than required (" <<
                poses.size()<<", required "<<idx <<")");
            return false;
        }
        pose.pose = poses[idx];
    }
    else
    {
        ROS_ERROR_STREAM("Unknown mode of Object::primitive_origin or Object::mesh_origin: "<<idx);
        return false;
    }
    pose.header = object_header;
    return true;
}
geometry_msgs::Point ObjectFunctions::getAveragePointFrom(const std::vector<geometry_msgs::Pose>& poses)
{
    Eigen::Vector3d p;
    for (int i=0; i < poses.size(); ++i)
    {
        Eigen::Vector3d _p;
        const geometry_msgs::Pose& pi = poses[i];
        tf::pointMsgToEigen(pi.position,_p);
        p+=_p;
    }
    geometry_msgs::Point ret;
    tf::pointEigenToMsg(p,ret);
    return ret;
}
