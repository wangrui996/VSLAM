#include "pose_local_parameterization.h"

//参数块
//delta 增量
//x_plus_delta "加完"之后的量
bool PoseLocalParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    //做一个eigen映射 将double数组x映射成Vector3d，就能像操作eigen对象一样操作double数组
    Eigen::Map<const Eigen::Vector3d> _p(x);//double数组前三维？平移部分
    
    Eigen::Map<const Eigen::Quaterniond> _q(x + 3);//四元数
    //增量，映射成Vector3d，取得是double数组前三个元素，也就是位移增量
    Eigen::Map<const Eigen::Vector3d> dp(delta);
    //旋转的变换量(一个旋转向量)取出后映射成Vector3d，调用deltaQ接口将拿到的旋转向量(很小)转换成四元数
    Eigen::Quaterniond dq = Utility::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));
    //将广义加法完成后的量映射成eigen量p和q，后面直接用eigen操作改变p和q即可
    Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
    Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);

    p = _p + dp; //直接加
    q = (_q * dq).normalized();

    return true;
}
bool PoseLocalParameterization::ComputeJacobian(const double *x, double *jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
    j.topRows<6>().setIdentity();
    j.bottomRows<1>().setZero();

    return true;
}
