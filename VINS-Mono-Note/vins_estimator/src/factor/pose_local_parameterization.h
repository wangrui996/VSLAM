#pragma once

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include "../utility/utility.h"

// ceres solver定义局部参数块，需要继承下面4个函数
class PoseLocalParameterization : public ceres::LocalParameterization
{
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;
    virtual bool ComputeJacobian(const double *x, double *jacobian) const;
    virtual int GlobalSize() const { return 7; }; //7：平移3+四元数4
    virtual int LocalSize() const { return 6; }; //6：实际自由度 平移3 姿态3
};
