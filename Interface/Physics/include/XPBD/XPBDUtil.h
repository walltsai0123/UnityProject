#pragma once

#ifndef _XPBD_UTIL_H_
#define _XPBD_UTIL_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

template <typename ScalarType, int Options>
Eigen::Quaternion<ScalarType, Options> operator*(ScalarType a, const Eigen::Quaternion<ScalarType, Options> &q)
{
    return Eigen::Quaternion<ScalarType, Options>(a * q.w(), a * q.x(), a * q.y(), a * q.z());
}

template <typename ScalarType, int Options>
Eigen::Quaternion<ScalarType, Options> operator+(const Eigen::Quaternion<ScalarType, Options> &q1, const Eigen::Quaternion<ScalarType, Options> &q2)
{
    return Eigen::Quaternion<ScalarType, Options>(q1.w() + q2.w(), q1.x() + q2.x(), q1.y() + q2.y(), q1.z() + q2.z());
}

#endif
