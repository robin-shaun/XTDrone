#ifndef OBJECT_MSGS_TOOLS_OBJECTTFBROADCASTER_H
#define OBJECT_MSGS_TOOLS_OBJECTTFBROADCASTER_H

#include <ros/ros.h>
#include <object_msgs/Object.h>
#include <object_msgs/RegisterObject.h>
#include <tf/transform_broadcaster.h>
#include <boost/thread.hpp>

namespace object_msgs_tools {

/**
 * Subscribes to object_msgs/Object topic or uses the object_msgs/ObjectInfo service
 * to query information about objects, and then broadcasts the information as /tf transform.
 * Only objects which have been explicitly registered with registerObject() are processed.
 * Only objects which arrive or are queried with the service and have a Object.header.frame_id
 * which is **not empty** are considered valid and are processed.
 *
 * Refer to the constructor for more information about how it works.
 *
 * \author Jennifer Buehler
 * \date March 2016
 */
class ObjectTFBroadcaster
{
public:

    /**
     * Constructor. Parameters describe how the ObjectTFBRoadcaster works.
     * \param register_object_service the name under which a service of type object_msgs/RegisterObject
     *      is going to be provided which can be used to add objects to be published wiht /tf.
     * \param query_object_info_rate when using the object_msgs/ObjectInfo service
     *      to obtain object information, this is the rate at which this is done.
     *      If no service is to be used, and instead it should be relied on
     *      messages published on \e object_topic only, then \e object_service
     *      should be left empty, and \e query_object_info_rate will be ignored.
     * \param publish_tf_rate rate at which to publish information to /tf. This may differ
     *      from the \e query_object_info_rate, i.e. it may be set higer, in order
     *      to keep refreshing the timestamps of the /tf tree. Because it is expected
     *      that objects don't change their state very often, \e query_object_info_rate
     *      (or the rate at which object messages arrive at \e object_topic) can be lower.
     * \param object_topic the topic at which object_msgs/Object information is being
     *      published. Can be left empty to instead rely on \e object_service
     * \param object_service the object_msgs/ObjectInfo service where information
     *      about objects can be queried. Can be left empty to rely on messages being
     *      published on \e object_topic. The service is queried at \e query_object_info_rate.
     */
    ObjectTFBroadcaster(
        ros::NodeHandle& n,
        const std::string& register_object_service,
        unsigned int publish_tf_rate,
        unsigned int query_object_info_rate,
        const std::string& object_topic,
        const std::string& object_service);

    bool isRegistered(const std::string& name) const; 

    /**
     * return code as in object_msgs::RegisterObject
     */
    int registerObject(const std::string& name, bool printMsgs = true); 

private:

    /**
     * service callback
     */
    bool registerObjectService(object_msgs::RegisterObject::Request &req,
                               object_msgs::RegisterObject::Response &res);

    /**
     * Uses the service to request object information (only pose)
     */     
    bool queryObjectPose(const std::string& name, object_msgs::Object& obj, bool printErrors = true);

    /**
     * This transform (pose) holds for the target frame frame_name
     */
    void sendTF(const geometry_msgs::PoseStamped& pose, const std::string& frame_name);

    /**
     * Updates information of this object in the map.
     */
    bool updateObject(const object_msgs::Object& obj);
   
    /**
     * Callback for subscriber to object_msgs/Object
     */ 
    void objectCallback(const object_msgs::Object& obj);

    /**
     * Event loop for publishing tf
     */
    void publishTFEvent(const ros::TimerEvent& e);

    /**
     * Event loop for querying the newest object information
     */
    void queryObjectPoseEvent(const ros::TimerEvent& e);

    // all object poses which have been registered
    // and will be continuously re-published on /tf. 
    std::map<std::string, geometry_msgs::PoseStamped> obj_poses;
    mutable boost::recursive_mutex obj_poses_mutex;

    ros::ServiceServer register_object_srv;
    ros::ServiceClient object_info_srv;
    ros::Subscriber object_info_sub;

    ros::NodeHandle node;
            
    tf::TransformBroadcaster tf_broadcaster;
    ros::Timer publish_tf_timer;
    ros::Timer query_object_info_timer;
};

}

#endif  // OBJECT_MSGS_TOOLS_OBJECTTFBROADCASTER_H
