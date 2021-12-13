
#include <CostFunctor.h>
#include <iostream>

using namespace std;

int main(int agrc, char** agrv)
{

    //优化问题初值  
    double initial_x = 0.0;
    double x  = initial_x;

    //构建问题,实例化 ceres::Problem 
    ceres::Problem problem;

    //设置损失函数  自动求导方式计算雅可比矩阵 
    //第一个1是残差的维度； 第二个1是输入维度，即待寻优参数x的维度 ?
    ceres::CostFunction* cost_fution = new ceres::AutoDiffCostFunction<cost_fution, 1, 1>(new CostFunctor);  

    //添加残差项  1.上一步实例化后的代价函数； 2.核函数  3.待优化变量地址
    problem.AddResidualBlock(cost_fution, nullptr, &x);  

    //配置并运行slover 
    ceres::Solver::Options options;
    //配置增量方程的解法  这里设置为QR分解  
    options.linear_solver_type = ceres::DENSE_QR; 
    //是否输出到cout 计算过程打印到屏幕上  
    options.minimizer_progress_to_stdout = true;
    //options.max_solver_time_in_seconds = 0.2; 最大迭代时间 s
    //优化信息  
    ceres::Solver::Summary summary;
    ceres::Solve(option, &problem, &summary);

    //输出优化的简要信息
    cout << summary.BriefReport() << endl;

    //最终结果
    cout << "x初始值：" << initial_x << endl 
         << "迭代到：->" << x << endl;

    return 0;
}












