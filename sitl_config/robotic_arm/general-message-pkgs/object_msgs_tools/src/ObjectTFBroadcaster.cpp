#include <object_msgs_tools/ObjectTFBroadcaster.h>
#include <object_msgs_tools/ObjectFunctions.h>
#include <object_msgs/ObjectInfo.h>

using object_msgs_tools::ObjectTFBroadcaster;

ObjectTFBroadcaster::ObjectTFBroadcaster(
    ros::NodeHandle& n,
    const std::string& register_object_service,
    unsigned int publish_tf_rate,
    unsigned int query_object_info_rate,
    const std::string& object_topic,
    const std::string& object_service): 
    node(n)
{
    if (object_service.empty() && object_topic.empty())
    {
        ROS_ERROR("Can't use ObjetTFBroadcaster if neither service nor object info topic is set");
        throw std::runtime_error("Can't use ObjetTFBroadcaster if neither service nor object info topic is set");
    }
    
    ros::Rate rate(publish_tf_rate);
    publish_tf_timer = node.createTimer(rate, &ObjectTFBroadcaster::publishTFEvent, this);
    if (!register_object_service.empty())
    {
        ROS_INFO_STREAM("Advertising service "<<register_object_service);
        register_object_srv = node.advertiseService(register_object_service, &ObjectTFBroadcaster::registerObjectService,this);
    }
    
    if (!object_service.empty())
    {
        ros::Rate qRate(query_object_info_rate);
        query_object_info_timer=node.createTimer(qRate,&ObjectTFBroadcaster::queryObjectPoseEvent, this);
        query_object_info_timer.start();
        object_info_srv = node.serviceClient<object_msgs::ObjectInfo>(object_service);
    }
    if (!object_topic.empty())
        object_info_sub = node.subscribe(object_topic,1000,&ObjectTFBroadcaster::objectCallback, this);
}

bool ObjectTFBroadcaster::isRegistered(const std::string& name) const
{
    obj_poses_mutex.lock();
    bool exists= obj_poses.find(name) != obj_poses.end();
    obj_poses_mutex.unlock();
    return exists;
}

bool ObjectTFBroadcaster::registerObjectService(object_msgs::RegisterObject::Request &req,
                                                object_msgs::RegisterObject::Response &res)
{
    ROS_INFO_STREAM("Calling ObjectTFBroadcaster service with "<<req.name);
    res.success = registerObject(req.name, true);
    // always return success because none of the errors
    // are fatal, the object will probably still be updated
    // even if the information was not available at the time.
    // It was still added (or already exists).
    return true;
}


int ObjectTFBroadcaster::registerObject(const std::string& name, bool printMsgs)
{
    object_msgs::Object obj;
    // Get initial information of the object.
    // If this fails it doesn't matter, the first pose inserted will be invalid
    queryObjectPose(name,obj, false);
    geometry_msgs::PoseStamped pose;
    if (!ObjectFunctions::getObjectPose(obj, pose)){
        if (printMsgs) ROS_WARN_STREAM("ObjectTFBroadcaster: Could not get pose for object '"<<name<<"'");
        return object_msgs::RegisterObject::Response::ERROR_INFO;
    }

    obj_poses_mutex.lock();
    bool success = obj_poses.insert(std::make_pair(name,pose)).second;
    obj_poses_mutex.unlock();
    if (!success)
    {
        if (printMsgs)
            ROS_WARN_STREAM("Object " << name << 
                " could not be added in ObjectTFBroadcaster because it was already registered.");
        return object_msgs::RegisterObject::Response::EXISTS;
    }
    return object_msgs::RegisterObject::Response::SUCCESS;
}

 
bool ObjectTFBroadcaster::queryObjectPose(const std::string& name, object_msgs::Object& obj, bool printErrors)
{
    if (object_info_srv.getService().empty())
    {
        if (printErrors) ROS_ERROR("ObjectTFBroadcaster: Service to request object_msgs/Object is not running.");
        return false;
    }
    object_msgs::ObjectInfo srv;
    srv.request.name = name;
    srv.request.get_geometry = false;
    if (object_info_srv.call(srv)){
        //ROS_INFO("Result:");
        //std::cout<<srv.response<<std::endl;
    }else{
        if (printErrors) ROS_ERROR("ObjectTFBroadcaster: Failed to call service to obtain object info");
        return false;
    }
    obj=srv.response.object;
    return true;
}

void ObjectTFBroadcaster::sendTF(const geometry_msgs::PoseStamped& pose, const std::string& frame_name){
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) );
    tf::Quaternion q(pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w);
    transform.setRotation(q);
    // ROS_INFO_STREAM("Send tf in "<<frame_name<<". "); //<<std::endl<<pose);
    tf_broadcaster.sendTransform(tf::StampedTransform(transform, pose.header.stamp, pose.header.frame_id, frame_name));
    // ROS_INFO("TF sent.");
}


bool ObjectTFBroadcaster::updateObject(const object_msgs::Object& obj)
{
    // ROS_INFO_STREAM("ObjectTFBroadcaster: Got object "<<obj);
    geometry_msgs::PoseStamped pose;
    if (!ObjectFunctions::getObjectPose(obj, pose)){
        ROS_ERROR("ObjectTFBroadcaster: Could not get pose");
        return false;
    }
    std::map<std::string, geometry_msgs::PoseStamped>::iterator it;
    obj_poses_mutex.lock();
    it = obj_poses.find(obj.name);
    bool success = (it != obj_poses.end());
    if (success) it->second=pose;
    obj_poses_mutex.unlock();
    if (!success)
    {
        ROS_ERROR_STREAM("ObjectTFBroadcaster: Could not update object "
            <<obj.name<<" because it was not registered. Call registerObject() to add it.");
    }
    return success;
}

void ObjectTFBroadcaster::objectCallback(const object_msgs::Object& obj){
    updateObject(obj);
}

void ObjectTFBroadcaster::publishTFEvent(const ros::TimerEvent& e){
    // ROS_INFO("TF");
    obj_poses_mutex.lock();
    std::map<std::string, geometry_msgs::PoseStamped>::iterator it;
    for (it = obj_poses.begin(); it!=obj_poses.end(); ++it)
    {
        it->second.header.stamp=ros::Time::now();
        // ROS_INFO_STREAM("Sending TF for "<<it->first);//<<": "<<it->second);
        sendTF(it->second,it->first);    
    }
    obj_poses_mutex.unlock();
}

void ObjectTFBroadcaster::queryObjectPoseEvent(const ros::TimerEvent& e)
{
    // ROS_INFO("Query");
    obj_poses_mutex.lock();
    std::map<std::string, geometry_msgs::PoseStamped>::iterator it;
    for (it = obj_poses.begin(); it!=obj_poses.end(); ++it)
    {
        object_msgs::Object obj;
        if (!queryObjectPose(it->first,obj,true)) continue;
        updateObject(obj);
    }
    obj_poses_mutex.unlock();
}
