#ifndef _mm_pdoran_event_h_
#define _mm_pdoran_event_h_

#include <mutex>
#include <condition_variable>

namespace anki
{

/**
 * @brief Synchronization object that allows a thread to signal to other threads that an event has happened.
 */
class Event
{
public:
  Event() : _triggered(false), _mutex(), _condition()
  {
  }

  void set()
  {
    std::lock_guard<std::mutex> lk(_mutex);
    _triggered = true;
    _condition.notify_all();
  }

  void reset()
  {
    std::lock_guard<std::mutex> lk(_mutex);
    _triggered = false;
  }

  void wait(){
    std::unique_lock<std::mutex> lk(_mutex);
    _condition.wait(lk,[&]{return this->_triggered; });
  };

private:
  volatile bool _triggered;
  std::mutex _mutex;
  std::condition_variable _condition;
};

}

#endif