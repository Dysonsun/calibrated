/*!
* \file rigid_transform.h
* \brief 位姿及位姿转换
*
*该文件对Eigen 库的Matrix Quaterniond　等进行封装，表示车辆的位姿，进行位姿的转换
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

#ifndef CARTOGRAPHER_TRANSFORM_RIGID_TRANSFORM_H_
#define CARTOGRAPHER_TRANSFORM_RIGID_TRANSFORM_H_

#include <iostream>
#include <string>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "common/lua_parameter_dictionary.h"
#include "common/math.h"
#include "common/port.h"
#include "common/time.h"

namespace transform {
/// \brief 二维位姿变换类
///
/// 存储二维位姿（旋转及平移），进行位姿的变换
template <typename FloatType>
class Rigid2 {
 public:
  using Vector = Eigen::Matrix<FloatType, 2, 1>;
  using Rotation2D = Eigen::Rotation2D<FloatType>;

  Rigid2()
      : translation_(Vector::Identity()), rotation_(Rotation2D::Identity()) {}
  Rigid2(const Vector& translation, const Rotation2D& rotation)
      : translation_(translation), rotation_(rotation) {}
  Rigid2(const Vector& translation, const double rotation)
      : translation_(translation), rotation_(rotation) {}

  static Rigid2 Rotation(const double rotation) {
    return Rigid2(Vector::Zero(), rotation);
  }

  static Rigid2 Rotation(const Rotation2D& rotation) {
    return Rigid2(Vector::Zero(), rotation);
  }

  static Rigid2 Translation(const Vector& vector) {
    return Rigid2(vector, Rotation2D::Identity());
  }

  static Rigid2<FloatType> Identity() {
    return Rigid2<FloatType>(Vector::Zero(), Rotation2D::Identity());
  }

  template <typename OtherType>
  Rigid2<OtherType> cast() const {
    return Rigid2<OtherType>(translation_.template cast<OtherType>(),
                             rotation_.template cast<OtherType>());
  }

  const Vector& translation() const { return translation_; }

  Rotation2D rotation() const { return rotation_; }

  double normalized_angle() const {
    return common::NormalizeAngleDifference(rotation().angle());
  }

  Rigid2 inverse() const {
    const Rotation2D rotation = rotation_.inverse();
    const Vector translation = -(rotation * translation_);
    return Rigid2(translation, rotation);
  }

  string DebugString() const {
    string out;
    out.append("{ t: [");
    out.append(std::to_string(translation().x()));
    out.append(", ");
    out.append(std::to_string(translation().y()));
    out.append("], r: [");
    out.append(std::to_string(rotation().angle()));
    out.append("] }");
    return out;
  }

 private:
  Vector translation_;
  Rotation2D rotation_;
};

template <typename FloatType>
Rigid2<FloatType> operator*(const Rigid2<FloatType>& lhs,
                            const Rigid2<FloatType>& rhs) {
  return Rigid2<FloatType>(
      lhs.rotation() * rhs.translation() + lhs.translation(),
      lhs.rotation() * rhs.rotation());
}

template <typename FloatType>
typename Rigid2<FloatType>::Vector operator*(
    const Rigid2<FloatType>& rigid,
    const typename Rigid2<FloatType>::Vector& point) {
  return rigid.rotation() * point + rigid.translation();
}

// This is needed for gmock.
template <typename T>
std::ostream& operator<<(std::ostream& os,
                         const transform::Rigid2<T>& rigid) {
  os << rigid.DebugString();
  return os;
}

using Rigid2d = Rigid2<double>;
using Rigid2f = Rigid2<float>;

/// \brief 三维位姿变换类
///
/// 存储三维位姿（旋转及平移），进行位姿的变换
template <typename FloatType>
class Rigid3 {
 public:
  using Vector = Eigen::Matrix<FloatType, 3, 1>;
  using Quaternion = Eigen::Quaternion<FloatType>;
  using AngleAxis = Eigen::AngleAxis<FloatType>;

  Rigid3()
      : translation_(Vector::Identity()), rotation_(Quaternion::Identity()) {}
  Rigid3(const Vector& translation, const Quaternion& rotation)
      : translation_(translation), rotation_(rotation) {}
  Rigid3(const Vector& translation, const AngleAxis& rotation)
      : translation_(translation), rotation_(rotation) {}

  static Rigid3 Rotation(const AngleAxis& angle_axis) {
    return Rigid3(Vector::Zero(), Quaternion(angle_axis));
  }

  static Rigid3 Rotation(const Quaternion& rotation) {
    return Rigid3(Vector::Zero(), rotation);
  }

  static Rigid3 Translation(const Vector& vector) {
    return Rigid3(vector, Quaternion::Identity());
  }

  static Rigid3<FloatType> Identity() {
    return Rigid3<FloatType>(Vector::Zero(), Quaternion::Identity());
  }
  /// \brief 强制类型转换
  ///
  /// \return 新类型的对象
  template <typename OtherType>
  Rigid3<OtherType> cast() const {
    return Rigid3<OtherType>(translation_.template cast<OtherType>(),
                             rotation_.template cast<OtherType>());
  }

  const Vector& translation() const { return translation_; }
  const Quaternion& rotation() const { return rotation_; }

  Rigid3 inverse() const {
    const Quaternion rotation = rotation_.conjugate();
    const Vector translation = -(rotation * translation_);
    return Rigid3(translation, rotation);
  }
  /// \brief 姿态插值
  ///
  /// 同时进行位移和姿态的插值
  /// \param rate　插值比率
  /// \param other　插值的后基准
  /// \return 插值结果
  Rigid3 slerp(double rate,const Rigid3 other) const {

    const Quaternion rotation = this->rotation().slerp(rate,other.rotation());
    const Vector translation = this->translation() * (1-rate) + other.translation()*rate;
    return Rigid3(translation, rotation);
  }

  string DebugString() const {
    string out;
    out.append("{ t: [");
    out.append(std::to_string(translation().x()));
    out.append(", ");
    out.append(std::to_string(translation().y()));
    out.append(", ");
    out.append(std::to_string(translation().z()));
    out.append("], q: [");
    out.append(std::to_string(rotation().w()));
    out.append(", ");
    out.append(std::to_string(rotation().x()));
    out.append(", ");
    out.append(std::to_string(rotation().y()));
    out.append(", ");
    out.append(std::to_string(rotation().z()));
    out.append("] }");
    return out;
  }

 private:
  Vector translation_;
  Quaternion rotation_;
};

template <typename FloatType>
Rigid3<FloatType> operator*(const Rigid3<FloatType>& lhs,
                            const Rigid3<FloatType>& rhs) {
  return Rigid3<FloatType>(
      lhs.rotation() * rhs.translation() + lhs.translation(),
      (lhs.rotation() * rhs.rotation()).normalized());
}

template <typename FloatType>
typename Rigid3<FloatType>::Vector operator*(
    const Rigid3<FloatType>& rigid,
    const typename Rigid3<FloatType>::Vector& point) {
  return rigid.rotation() * point + rigid.translation();
}

// This is needed for gmock.
template <typename T>
std::ostream& operator<<(std::ostream& os,
                         const transform::Rigid3<T>& rigid) {
  os << rigid.DebugString();
  return os;
}

using Rigid3d = Rigid3<double>;
using Rigid3f = Rigid3<float>;

// Converts (roll, pitch, yaw) to a unit length quaternion. Based on the URDF
// specification http://wiki.ros.org/urdf/XML/joint.
Eigen::Quaterniond RollPitchYaw(double roll, double pitch, double yaw);

Eigen::Quaterniond YawPitchRoll(double yaw, double pitch, double roll);
Eigen::Vector3d toRollPitchYaw(const Eigen::Quaterniond& q);

// Returns an transform::Rigid3d given a 'dictionary' containing 'translation'
// (x, y, z) and 'rotation' which can either we an array of (roll, pitch, yaw)
// or a dictionary with (w, x, y, z) values as a quaternion.
Rigid3d FromDictionary(common::LuaParameterDictionary* dictionary);



struct posestamped{
  posestamped() = default;
  posestamped(common::Time time, const transform::Rigid3d& pose)
  :time(time)
  ,pose(pose){}

  common::Time time = common::Time::min();
  transform::Rigid3d pose = transform::Rigid3d::Identity();
};
}  // namespace transform

#endif  // CARTOGRAPHER_TRANSFORM_RIGID_TRANSFORM_H_
