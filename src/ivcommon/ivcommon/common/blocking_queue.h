/*!
* \file blocking_queue.h
* \brief 阻塞队列，可用于多线程间数据交流
*
*　阻塞队列，可用于多线程间数据交流
*
* \author The Cartographer Authors
* \version v1.2.1
* \date 2018/07/27
*/
/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_COMMON_BLOCKING_QUEUE_H_
#define CARTOGRAPHER_COMMON_BLOCKING_QUEUE_H_

#include <cstddef>
#include <deque>
#include <memory>

#include "common/mutex.h"
#include "common/port.h"
#include "common/time.h"
#include "glog/logging.h"

namespace common {

// A thread-safe blocking queue that is useful for producer/consumer patterns.
// 'T' must be movable.
/// \brief 阻塞队列，可用于多线程间数据交流
///
/// 阻塞队列，可用于多线程间数据交流
template <typename T>
class BlockingQueue {
 public:
  static constexpr size_t kInfiniteQueueSize = 0;

  // Constructs a blocking queue with infinite queue size.
  BlockingQueue() : BlockingQueue(kInfiniteQueueSize){}

  BlockingQueue(const BlockingQueue&) = delete;
  BlockingQueue& operator=(const BlockingQueue&) = delete;

  // Constructs a blocking queue with a size of 'queue_size'.
  explicit BlockingQueue(const size_t queue_size) : queue_size_(queue_size), request_to_end_(false) {}


  // Pushes a value onto the queue. Blocks if the queue is full.
  /// \brief 队列末端添加，但队列满时会阻塞
  ///
  /// \param t 待添加对象
  void Push(T t) {
    MutexLocker lock(&mutex_);
    lock.Await([this]() REQUIRES(mutex_) { return QueueNotFullCondition(); });
    deque_.push_back(std::move(t));
  }
  /// \brief 队列末端添加，但队列满时不阻塞，删除最前端数据后继续插入数据
  ///
  /// \param t 待添加对象
  void Push_Force(T t) {
    MutexLocker lock(&mutex_);
    lock.Await([this]() REQUIRES(mutex_) { return true; });
    if(!QueueNotFullCondition())
    	deque_.pop_front();
    deque_.push_back(std::move(t));
  }
  /// \brief 队列前端添加
  ///
  /// \param t 待添加对象
  void Push_Front(T t) {
    MutexLocker lock(&mutex_);
    lock.Await([this]() REQUIRES(mutex_) { return QueueNotFullCondition(); });
    deque_.push_front(std::move(t));
  }

  // Like push, but returns false if 'timeout' is reached.
  /// \brief 队列末端添加，但队列满时会阻塞，等待timeout时间后返回
  ///
  /// \param t 待添加对象
  /// \param timeout　阻塞最长时间
  bool PushWithTimeout(T t, const common::Duration timeout) {
    MutexLocker lock(&mutex_);
    if (!lock.AwaitWithTimeout(
            [this]() REQUIRES(mutex_) { return QueueNotFullCondition(); },
            timeout)) {
      return false;
    }
    deque_.push_back(std::move(t));
    return true;
  }

  // Pops the next value from the queue. Blocks until a value is available.
  /// \brief 弹出最前端数据，会阻塞等待队列非空或请求结束的信号
  ///
  T Pop() {
    MutexLocker lock(&mutex_);
    lock.Await([this]() REQUIRES(mutex_) { return request_to_end_||QueueNotEmptyCondition(); });

    if(request_to_end_)
    	return nullptr;
    T t = std::move(deque_.front());
    deque_.pop_front();
    return t;
  }

  void Clear() {
    MutexLocker lock(&mutex_);

    deque_.clear();
  }

  // Like Pop, but can timeout. Returns nullptr in this case.
  /// \brief 弹出最前端数据，但队列空时会阻塞，等待timeout时间后返回nullptr
  ///
  /// \param timeout　阻塞最长时间
  T PopWithTimeout(const common::Duration timeout) {
    MutexLocker lock(&mutex_);
    if (!lock.AwaitWithTimeout(
            [this]() REQUIRES(mutex_) { return QueueNotEmptyCondition(); },
            timeout)) {
      return nullptr;
    }
    T t = std::move(deque_.front());
    deque_.pop_front();
    return t;
  }

  // Returns the next value in the queue or nullptr if the queue is empty.
  // Maintains ownership. This assumes a member function get() that returns
  // a pointer to the given type R.
  /// \brief 获取最前端数据的指针或ｎｕｌｌｐｔｒ
  ///
  template <typename R>
  const R* Peek() {
    MutexLocker lock(&mutex_);
    if (deque_.empty()) {
      return nullptr;
    }
    return deque_.front().get();
  }
  /// \brief 获取最后端数据的指针或ｎｕｌｌｐｔｒ
  ///
  template <typename R>
  const R* PeekBack() {
    MutexLocker lock(&mutex_);
    if (deque_.empty()) {
      return nullptr;
    }
    return deque_.front().get();
  }

  // Returns the number of items currently in the queue.
  /// \brief 队列大小
  ///
  size_t Size() {
    MutexLocker lock(&mutex_);
    return deque_.size();
  }

  /// \brief 结束阻塞中的函数
  ///
  void
  stopQueue ()
  {
	MutexLocker lock(&mutex_);
    request_to_end_ = true;
  }
 private:
  // Returns true iff the queue is not empty.
  bool QueueNotEmptyCondition() REQUIRES(mutex_) { return !deque_.empty(); }

  // Returns true iff the queue is not full.
  bool QueueNotFullCondition() REQUIRES(mutex_) {
    return queue_size_ == kInfiniteQueueSize || deque_.size() < queue_size_;
  }

  Mutex mutex_;
  const size_t queue_size_ GUARDED_BY(mutex_);
  std::deque<T> deque_ GUARDED_BY(mutex_);
  bool request_to_end_;
};

}  // namespace common

#endif  // CARTOGRAPHER_COMMON_BLOCKING_QUEUE_H_
