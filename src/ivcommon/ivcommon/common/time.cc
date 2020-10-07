/*!
* \file time.cc
* \brief 时间相关的操作
*
*　时间相关的操作
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

#include "common/time.h"

#include <string>


namespace common {
/// \brief double s时间转换为时间段
/// \param seconds s时间
/// \return 时间段
Duration FromSeconds(const double seconds) {
  return std::chrono::duration_cast<Duration>(
      std::chrono::duration<double>(seconds));
}
/// \brief 时间段转double s时间
/// \param duration 时间段
/// \return s时间
double ToSeconds(const Duration duration) {
  return std::chrono::duration_cast<std::chrono::duration<double>>(duration)
      .count();
}

Time FromUniversal(const int64 ticks) { return Time(Duration(ticks)); }

int64 ToUniversal(const Time time) { return time.time_since_epoch().count(); }

/// \brief 获取当前时间
///
/// \return 时间
Time now() //ros's "now()"  is based on the time of /clock  ,this "now" is based on system time
{
	return FromUniversal(std::chrono::duration_cast<Duration>(std::chrono::system_clock::now().time_since_epoch()).count()
			+ kUtsEpochOffsetFromUnixEpochInSeconds*10000000ll);
}

std::ostream& operator<<(std::ostream& os, const Time time) {
  os << std::to_string(ToUniversal(time));
  return os;
}

common::Duration FromMilliseconds(const int64 milliseconds) {
  return std::chrono::duration_cast<Duration>(
      std::chrono::milliseconds(milliseconds));
}

}  // namespace common
