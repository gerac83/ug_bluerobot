/**
 *  \brief Concurent Queue implementation for multi-consumer / mutli-producer
 *  \Author: Anthony Williams
 *  \Details: http://www.justsoftwaresolutions.co.uk/threading/implementing-a-thread-safe-queue-using-condition-variables.html
 */

#include <queue>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/graph/buffer_concepts.hpp>

#ifndef CONCURENT_QUEUE_H
#define CONCURENT_QUEUE_H

template<typename Data>
class ConcurentQueue {
private:
    std::queue<Data> the_queue;
    mutable boost::mutex the_mutex;
    boost::condition_variable the_condition_variable;
    bool kill_required;
public:

    ConcurentQueue() : kill_required(false) {}

    /** \brief Request to end waiting functions */
    void kill() {
        boost::mutex::scoped_lock lock(the_mutex);
        kill_required = true;
        lock.unlock();
        the_condition_variable.notify_all();
    }
    
    /** \brief Push all data at once and then notify all thread */
    void push_vector(const std::vector<Data>& data) {
        boost::mutex::scoped_lock lock(the_mutex);
        for(unsigned int i = 0; i < data.size(); ++i) {
            the_queue.push(data[i]);
        }
        lock.unlock();
        the_condition_variable.notify_all();
    }
    
    /** \brief Clear all data in a queue by replacing queue by empty one*/
    void clear() {
        boost::mutex::scoped_lock lock(the_mutex);
        std::queue<Data> empty;
        std::swap(the_queue, empty);
    }

    void push(Data const& data) {
        boost::mutex::scoped_lock lock(the_mutex);
        the_queue.push(data);
        lock.unlock();
        the_condition_variable.notify_one();
    }

    bool empty() const {
        boost::mutex::scoped_lock lock(the_mutex);
        return the_queue.empty();
    }

    bool try_pop(Data& popped_value) {
        boost::mutex::scoped_lock lock(the_mutex);
        if(the_queue.empty()) {
            return false;
        }

        popped_value = the_queue.front();
        the_queue.pop();
        return true;
    }

    /** \brief Wait for data and pop
     *  \return false if kill was called */
    bool wait_and_pop(Data& popped_value) {
        boost::mutex::scoped_lock lock(the_mutex);
        while(the_queue.empty()) {
            the_condition_variable.wait(lock);
            if(kill_required)
                return false;
        }

        popped_value = the_queue.front();
        the_queue.pop();
        return true;
    }

};

#endif //CONCURENT_QUEUE
