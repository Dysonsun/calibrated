/*!
* \file thread_pool.cc
* \brief 线程池操作
*
*　线程池操作
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

#include "common/thread_pool.h"

#include <unistd.h>
#include <algorithm>
#include <chrono>
#include <numeric>

#include "glog/logging.h"

namespace common {

/// \brief 最多开劈的线程数
///
/// \param num_threads　线程数
ThreadPool::ThreadPool(int num_threads) {
  MutexLocker locker(&mutex_);
  for (int i = 0; i != num_threads; ++i) {
    pool_.emplace_back([this]() { ThreadPool::DoWork(); });
  }
}

ThreadPool::~ThreadPool() {
  {
    MutexLocker locker(&mutex_);
    CHECK(running_);
    running_ = false;
    CHECK_EQ(work_queue_.size(), 0);
  }
  for (std::thread& thread : pool_) {
    thread.join();
  }
}

/// \brief 在线程池中增加待运行函数
///
/// \param work_item　函数
void ThreadPool::Schedule(const std::function<void()>& work_item) {
  MutexLocker locker(&mutex_);
  CHECK(running_);
  work_queue_.push_back(work_item);
}

/// \brief 在线程池中的运行/待运行函数数目
///
/// \param work_item　函数
int ThreadPool::DequeSize()
{
	  return work_queue_.size();
}

void ThreadPool::DoWork() {
#ifdef __linux__
  // This changes the per-thread nice level of the current thread on Linux. We
  // do this so that the background work done by the thread pool is not taking
  // away CPU resources from more important foreground threads.
  CHECK_NE(nice(10), -1);
#endif
  for (;;) {
    std::function<void()> work_item;
    {
      MutexLocker locker(&mutex_);
      locker.Await([this]() REQUIRES(mutex_) {
        return !work_queue_.empty() || !running_;
      });
      if (!work_queue_.empty()) {
        work_item = work_queue_.front();
        work_queue_.pop_front();
      } else if (!running_) {
        return;
      }
    }
    CHECK(work_item);
    work_item();
  }
}

}  // namespace common

