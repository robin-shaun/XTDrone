/* Copyright 2014-2015 The MathWorks, Inc. */

#ifndef _SLROS_GENERIC_PUBSUB_H_
#define _SLROS_GENERIC_PUBSUB_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/callback_queue.h>

extern ros::NodeHandle * SLROSNodePtr;  ///< The global node handle that is used by all ROS entities in the model

/**
* Class for subscribing to ROS messages in C++.
* 
* This class is used by code generated from the Simulink ROS
* subscriber blocks and is templatized by the ROS message type and
* Simulink bus type.
*/
template <class MsgType, class BusType>
class SimulinkSubscriber
{
public:
    void subscriberCallback(const boost::shared_ptr<MsgType const>&);
    void createSubscriber(std::string const& topic, uint32_t queueSize);
    bool getLatestMessage(BusType* busPtr); // returns true iff message is new

private:
    boost::shared_ptr<ros::CallbackQueue>   _customCallbackQueuePtr;
    ros::Subscriber                         _subscriber;
    bool                                    _newMessageReceived;
    boost::shared_ptr<MsgType const>        _lastMsgPtr;
};

/**
* Callback that is triggered when a new message is received
*
* This function will store the message as latest message.
* @param msgPtr The received message
*/
template <class MsgType, class BusType>
void SimulinkSubscriber<MsgType,BusType>::subscriberCallback(const boost::shared_ptr<MsgType const>& msgPtr) 
{
    _lastMsgPtr = msgPtr; // copy the shared_ptr
    _newMessageReceived = true;
}

/**
* Create a C++ subscriber object
*
* @param topic The topic name to subscribe to
* @param queueSize The length of incoming message queue
*/
template <class MsgType, class BusType>
void SimulinkSubscriber<MsgType,BusType>::createSubscriber(std::string const& topic, uint32_t queueSize)
{
    _customCallbackQueuePtr.reset( new ros::CallbackQueue() );

    ros::SubscribeOptions opts;
    opts.init<MsgType>(topic, queueSize, 
        boost::bind(&SimulinkSubscriber<MsgType,BusType>::subscriberCallback, this, _1));
    opts.callback_queue = _customCallbackQueuePtr.get();

    _subscriber = SLROSNodePtr->subscribe(opts);
}

/** 
* Get the latest received message
*
* @param busPtr Simulink bus structure that should be populated with message contents
* @return =TRUE, then a new message has been received and *busPtr holds the newly-received message. 
* =FALSE when a new message has not been received and *busPtr is unchanged.
*/
template <class MsgType, class BusType>
bool SimulinkSubscriber<MsgType,BusType>::getLatestMessage(BusType* busPtr) 
{
    _customCallbackQueuePtr->callOne(); 

    if (_newMessageReceived) {
        convertToBus(busPtr, _lastMsgPtr.get());
        _newMessageReceived = false;
        return true; // message is new
    } else {
        return false; // message is not new
    }
}


/**
* Class for publishing ROS messages in C++.
* 
* This class is used by code generated from the Simulink ROS
* publisher blocks and is templatized by the ROS message type and
* Simulink bus type.
*/
template <class MsgType, class BusType>
class SimulinkPublisher
{

public:
    void createPublisher(std::string const& topic, uint32_t queueSize);
    void publish(BusType* busPtr);

private:
    ros::Publisher  _publisher; ///< The ROS publisher object
    MsgType         _msg;       ///< A prototype of the message to publish (will be filled based on Simulink bus structure)
};


/**
* Create a publisher to a topic
*
* @param topic The name of the topic to advertise
* @param queueSize The length of outgoing publishing message queue
*/
template <class MsgType, class BusType>
void SimulinkPublisher<MsgType,BusType>::createPublisher(std::string const& topic, uint32_t queueSize)
{
    _publisher = SLROSNodePtr->advertise<MsgType>(topic, queueSize);
}

/**
* Publish a message
*
* @param busPtr Pointer to the bus structure for the outgoing message
*/
template <class MsgType, class BusType>
void SimulinkPublisher<MsgType,BusType>::publish(BusType* busPtr)
{
    convertFromBus(&_msg, busPtr);
    _publisher.publish(_msg);
}

#endif
