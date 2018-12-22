#ifndef MOTION_HPP
#define MOTION_HPP

#define _USE_MATH_DEFINES
#include <Eigen/Geometry>
#include <iostream>
#include <cstdlib>
#include <cmath>

bool isIdentity(const Eigen::Matrix3d &M);
bool isOrthogonal(const Eigen::Matrix3d &M);
double mixedProduct(const Eigen::Vector3d &x, const Eigen::Vector3d &y, const Eigen::Vector3d &z);

std::pair<Eigen::Vector3d, double> A2AngleAxis(const Eigen::Matrix3d &A);
Eigen::Matrix3d Rodriguez(const Eigen::Vector3d &p, const double phi);
Eigen::Vector3d A2Euler(const Eigen::Matrix3d &A);
Eigen::Matrix3d Euler2A(const Eigen::Vector3d &angles);
std::pair<Eigen::Vector3d, double> Q2AngleAxis(const Eigen::Quaterniond& q);
Eigen::Quaterniond AngleAxis2Q(const Eigen::Vector3d &p, const double phi);
Eigen::Quaterniond SlerpInterpolation(const Eigen::Quaterniond &q1,
                                      const Eigen::Quaterniond &q2,
                                      double tm, double t);

void writeEigenVector3d(const std::string &name, const Eigen::Vector3d &v);
void writeEigenMatrix3d(const std::string &name, const Eigen::Matrix3d &m);
void writeQuaternion(const std::string &name, const Eigen::Quaterniond &q);

#endif
