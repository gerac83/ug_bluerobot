// Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
//
// Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
// Institute:   Czech Technical University in Prague
// Created on:  Feb 13, 2014


#ifndef CLOPEMA_LIBS_TOPIC_UTILS_H_
#define CLOPEMA_LIBS_TOPIC_UTILS_H_

#include <ros/ros.h>
#include <boost/thread.hpp>

namespace clopema
{
namespace topic_utils
{

/**
 * A topic latch provides a way to get single messages without the need to
 * implement a callback method. It subscribes to a topic and latches the last
 * message. It can be used to get single message (similarly to
 * ros::wait_for_message) but with the possibility to drop some message before
 * returning. This is useful for example for camera topic where the first image
 * is usually not valid.
 *
 */
template<class T> class TopicLatch {

    ros::Subscriber _sub;
    T _msg;
    bool _hasMsg;
    boost::condition_variable _cond;
    boost::mutex _mut;


    public:
    /** \brief Initialize topic latch.  */
    TopicLatch(ros::NodeHandle nh, const std::string &topic) : _hasMsg(false)
    {
        _sub = nh.subscribe(topic, 1, &TopicLatch::input_cb, this);
    }

    /** \brief Destructor. */
    ~TopicLatch()
    {
    }

    /** \brief Get last received message. */
    T getLast()
    {
        if (!_hasMsg) {
            boost::unique_lock<boost::mutex> lock(_mut);
            _cond.wait(lock);
        }
        return _msg;
    }

    /** \brief Get next message, optionally drop some. */
    T getNext(int drop = 0)
    {
        boost::unique_lock<boost::mutex> lock(_mut);
        if (!_hasMsg) {
            _cond.wait(lock);
        }

        while (drop > 0) {
            _cond.wait(lock);
            drop--;
        }

        return _msg;
    }

    protected:

    void input_cb(const boost::shared_ptr<T const> &msg)
    {
        boost::unique_lock<boost::mutex> lock(_mut);
        _msg = *msg;
        _hasMsg = true;
        _cond.notify_all();
    }
};


/**
 * \brief Alternative for ros::wait_for_message with drop functionality.
 */
template<class T>
T wait_for_message(ros::NodeHandle nh, const std::string name, int drop = 0) {
    TopicLatch<T> tl(nh, name);
    return tl.getNext(drop);
}

} // End topic_utils
} // End clopema

#endif
