#include <gazebo_test_tools/FakeObjectRecognizer.h>
#include <object_msgs/RegisterObject.h>
#include <boost/thread.hpp>

#include <iostream>

#define DEFAULT_OBJECTS_TOPIC "world/objects"
#define DEFAULT_SERVICE_REQUEST_OBJECT_TOPIC "world/request_object"
#define DEFAULT_SERVICE_RECOGNISE_OBJECT_TOPIC "/recognize_object"
#define DEFAULT_SERVICE_REGISTER_OBJECT_TF_TOPIC "/register_object"
#define DEFAULT_PUBLISH_RECOGNISED_OBJECT_RATE 1

using gazebo_test_tools::FakeObjectRecognizer;

FakeObjectRecognizer::FakeObjectRecognizer() {

    ros::NodeHandle _node("/gazebo_test_tools");
    _node.param<std::string>("objects_topic", OBJECTS_TOPIC, DEFAULT_OBJECTS_TOPIC);
    ROS_INFO("Got objects topic name: <%s>", OBJECTS_TOPIC.c_str());

    _node.param<std::string>("request_object_service", SERVICE_REQUEST_OBJECT_TOPIC, DEFAULT_SERVICE_REQUEST_OBJECT_TOPIC);
    ROS_INFO("Got object service topic name: <%s>", SERVICE_REQUEST_OBJECT_TOPIC.c_str());
    
    _node.param<std::string>("recognize_object_service", SERVICE_RECOGNISE_OBJECT_TOPIC, DEFAULT_SERVICE_RECOGNISE_OBJECT_TOPIC);
    
    _node.param<std::string>("register_object_tf_service", SERVICE_REGISTER_OBJECT_TF_TOPIC, DEFAULT_SERVICE_REGISTER_OBJECT_TF_TOPIC);
    ROS_INFO("Got register object tf service topic name: <%s>", SERVICE_REGISTER_OBJECT_TF_TOPIC.c_str());

    int PUBLISH_RECOGNISED_OBJECT_RATE;
    _node.param<int>("publish_recognition_rate", PUBLISH_RECOGNISED_OBJECT_RATE, PUBLISH_RECOGNISED_OBJECT_RATE);

    if (!SERVICE_REQUEST_OBJECT_TOPIC.empty()) object_info_client = node.serviceClient<object_msgs::ObjectInfo>(SERVICE_REQUEST_OBJECT_TOPIC);
    if (!SERVICE_REGISTER_OBJECT_TF_TOPIC.empty()) register_object_tf_client = node.serviceClient<object_msgs::RegisterObject>(SERVICE_REGISTER_OBJECT_TF_TOPIC);

    recognize_object_srv = node.advertiseService(SERVICE_RECOGNISE_OBJECT_TOPIC, &FakeObjectRecognizer::recognizeObject,this);

    object_pub = node.advertise<object_msgs::Object>(OBJECTS_TOPIC, 100); 

    ros::Rate rate(PUBLISH_RECOGNISED_OBJECT_RATE);
    publishTimer=node.createTimer(rate,&FakeObjectRecognizer::publishRecognitionEvent, this);
}

FakeObjectRecognizer::~FakeObjectRecognizer() {
}

void FakeObjectRecognizer::publishRecognitionEvent(const ros::TimerEvent& e) {
    if (object_pub.getNumSubscribers()==0) return;
    
    boost::unique_lock<boost::mutex> lock(addedObjectsMtx);
    std::set<std::string>::iterator it;
    for (it=addedObjects.begin(); it!=addedObjects.end(); ++it )
    {
        object_msgs::Object object;
        // get the object information. Only the pose is required because
        // the shape has been published in recognizeObject() already and
        // it only needs to be published once when the object is first added.
        if (!queryObjectInfo(*it,object,false, true))
        {
            ROS_ERROR_STREAM("Could not find object "<<*it);
            continue;
        }
        object_pub.publish(object);
    }
}
    
bool FakeObjectRecognizer::recognizeObject(gazebo_test_tools::RecognizeGazeboObject::Request &req,
        gazebo_test_tools::RecognizeGazeboObject::Response &res)
{
    ROS_INFO_STREAM("Recognizing object "<<req.name); 
    
    res.success=true;
    boost::unique_lock<boost::mutex> lock(addedObjectsMtx);
    std::set<std::string>::iterator it = addedObjects.find(req.name);
    if (it!=addedObjects.end())
    {
        if (req.republish)
        { 
            ROS_WARN_STREAM("The object "<<req.name<<" was already set to being "
                 << "continously published. No need to call FakeObjectRecognizerService again.");
            return true; 
        }
        else  // switch off repbulishing for this object
        {
            ROS_INFO_STREAM("Removing object "<<req.name<<" from being re-published");
            addedObjects.erase(it);
            // now, publish the object information one last time, to
            // make sure this message is not misunderstood: it could have been
            // a caller which wants the message to be published once and wasn't aware
            // that the object was actually being re-published.
        }
    }
    if (req.republish) addedObjects.insert(req.name);

    object_msgs::Object object;
    // try this for a while, because sometimes when an object has just been
    // created / spawned, it may take a while for the service to return
    // information about it (before that it can't find it, e.g. if an object
    // is spawned in Gazebo and right after the recognition is triggered)
    float timeout = 3;
    float checkStep = 0.5;
    if (!waitForQueryObjectInfo(req.name,object,true, timeout, checkStep, false))
    {
        ROS_ERROR_STREAM("Could not find object "<<req.name);
        res.success=false;
        return true;
    }

    // ROS_INFO_STREAM("Publishing object "<<object);       
    object_pub.publish(object);

    // now, also register this object to be broadcasted in tf:
    object_msgs::RegisterObject srv;
	srv.request.name = object.name;
	if (register_object_tf_client.call(srv))
	{
		ROS_INFO("FakeObjectRecogniser: Register tf result:");
		std::cout<<srv.response<<std::endl;
	}
    return true;
}

bool FakeObjectRecognizer::waitForQueryObjectInfo(const std::string& name, object_msgs::Object& object,
    bool include_geometry, float timeout, float checkStep, bool printErrors)
{
    ros::Time startTime=ros::Time::now();
    float timeWaited = 0;
    while (timeWaited < timeout)
    {
        if (queryObjectInfo(name, object, include_geometry, printErrors)) return true;
        ros::Duration(checkStep).sleep();
        ros::Time currTime = ros::Time::now();
        timeWaited = (currTime-startTime).toSec();
    }
    return false;
}

bool FakeObjectRecognizer::queryObjectInfo(const std::string& name, object_msgs::Object& object, bool include_geometry, bool printErrors){
    object_msgs::ObjectInfo srv;
    srv.request.name=name;
    srv.request.get_geometry=include_geometry;
    if (!object_info_client.call(srv)){
        if (printErrors) ROS_ERROR("Could not get object %s because service request failed.",name.c_str());
        return false;
    }
    if (!srv.response.success) {
        if (printErrors) ROS_ERROR("Could not get object %s because it does not exist.",name.c_str());
        return false;
    }
    object=srv.response.object;
    return true;
}


