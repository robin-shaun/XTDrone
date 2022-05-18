/*
 * Copyright 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef ROS_PUBQUEUE_H
#define ROS_PUBQUEUE_H

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <deque>
#include <list>
#include <vector>

#include <ros/ros.h>


/// \brief Container for a (ROS publisher, outgoing message) pair.
/// We'll have queues of these.  Templated on a ROS message type.
template<class T>
class PubMessagePair
{
  public:
    /// \brief The outgoing message.
    T msg_;
    /// \brief The publisher to use to publish the message.
    ros::Publisher pub_;
    PubMessagePair(T& msg, ros::Publisher& pub) :
      msg_(msg), pub_(pub) {}
};

/// \brief A queue of outgoing messages.  Instead of calling publish() directly,
/// you can push() messages here to defer ROS serialization and locking.
/// Templated on a ROS message type.
template<class T>
class PubQueue
{
  public:
    typedef boost::shared_ptr<std::deque<boost::shared_ptr<
      PubMessagePair<T> > > > QueuePtr;
    typedef boost::shared_ptr<PubQueue<T> > Ptr;

  private:
    /// \brief Our queue of outgoing messages.
    QueuePtr queue_;
    /// \brief Mutex to control access to the queue.
    boost::shared_ptr<boost::mutex> queue_lock_;
    /// \brief Function that will be called when a new message is pushed on.
    boost::function<void()> notify_func_;

  public:
    PubQueue(QueuePtr queue,
             boost::shared_ptr<boost::mutex> queue_lock,
             boost::function<void()> notify_func) :
      queue_(queue), queue_lock_(queue_lock), notify_func_(notify_func) {}
    ~PubQueue() {}

    /// \brief Push a new message onto the queue.
    /// \param[in] msg The outgoing message
    /// \param[in] pub The ROS publisher to use to publish the message
    void push(T& msg, ros::Publisher& pub)
    {
      boost::shared_ptr<PubMessagePair<T> > el(new PubMessagePair<T>(msg, pub));
      boost::mutex::scoped_lock lock(*queue_lock_);
      queue_->push_back(el);
      notify_func_();
    }

    /// \brief Pop all waiting messages off the queue.
    /// \param[out] els Place to store the popped messages
    void pop(std::vector<boost::shared_ptr<PubMessagePair<T> > >& els)
    {
      boost::mutex::scoped_lock lock(*queue_lock_);
      while(!queue_->empty())
      {
        els.push_back(queue_->front());
        queue_->pop_front();
      }
    }
};

/// \brief A collection of PubQueue objects, potentially of different types.
/// This class is the programmer's interface to this queuing system.
class PubMultiQueue
{
  private:
    /// \brief List of functions to be called to service our queues.
    std::list<boost::function<void()> > service_funcs_;
    /// \brief Mutex to lock access to service_funcs_
    boost::mutex service_funcs_lock_;
    /// \brief If started, the thread that will call the service functions
    boost::thread service_thread_;
    /// \brief Boolean flag to shutdown the service thread if PubMultiQueue is destructed
    bool service_thread_running_;
    /// \brief Condition variable used to block and resume service_thread_
    boost::condition_variable service_cond_var_;
    /// \brief Mutex to accompany service_cond_var_
    boost::mutex service_cond_var_lock_;

    /// \brief Service a given queue by popping outgoing message off it and
    /// publishing them.
    template <class T>
    void serviceFunc(boost::shared_ptr<PubQueue<T> > pq)
    {
      std::vector<boost::shared_ptr<PubMessagePair<T> > > els;
      pq->pop(els);
      for(typename std::vector<boost::shared_ptr<PubMessagePair<T> > >::iterator it = els.begin();
          it != els.end();
          ++it)
      {
        (*it)->pub_.publish((*it)->msg_);
      }
    }

  public:
    PubMultiQueue() {}
    ~PubMultiQueue()
    {
      if(service_thread_.joinable())
      {
        service_thread_running_ = false;
        notifyServiceThread();
        service_thread_.join();
      }
    }

    /// \brief Add a new queue.  Call this once for each published topic (or at
    /// least each type of publish message).
    /// \return Pointer to the newly created queue, good for calling push() on.
    template <class T>
    boost::shared_ptr<PubQueue<T> > addPub()
    {
      typename PubQueue<T>::QueuePtr queue(new std::deque<boost::shared_ptr<PubMessagePair<T> > >);
      boost::shared_ptr<boost::mutex> queue_lock(new boost::mutex);
      boost::shared_ptr<PubQueue<T> > pq(new PubQueue<T>(queue, queue_lock, boost::bind(&PubMultiQueue::notifyServiceThread, this)));
      boost::function<void()> f = boost::bind(&PubMultiQueue::serviceFunc<T>, this, pq);
      {
        boost::mutex::scoped_lock lock(service_funcs_lock_);
        service_funcs_.push_back(f);
      }
      return pq;
    }

    /// \brief Service each queue one time.
    void spinOnce()
    {
      boost::mutex::scoped_lock lock(service_funcs_lock_);
      for(std::list<boost::function<void()> >::iterator it = service_funcs_.begin();
          it != service_funcs_.end();
          ++it)
      {
        (*it)();
      }
    }

    /// \brief Service all queues indefinitely, waiting on a condition variable
    /// in between cycles.
    void spin()
    {
      while(ros::ok() && service_thread_running_)
      {
        boost::unique_lock<boost::mutex> lock(service_cond_var_lock_);
        service_cond_var_.wait(lock);
        spinOnce();
      }
    }

    /// \brief Start a thread to call spin().
    void startServiceThread()
    {
      service_thread_running_ = true;
      service_thread_ = boost::thread(boost::bind(&PubMultiQueue::spin, this));
    }

    /// \brief Wake up the queue serive thread (e.g., after having pushed a
    /// message onto one of the queues).
    void notifyServiceThread()
    {
      service_cond_var_.notify_one();
    }
};

#endif
