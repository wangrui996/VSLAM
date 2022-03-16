# Vins-Fusion用到的Eigen  

## 注意  

ROS中使用的四元数顺序是x,y,z,w; 而Eigen中使用的四元数顺序是w,x,y,z 



1.FromTwoVectors()函数  

Eigen::Quaterniond::FromTwoVectors  

已知两个向量，其中一个向量由另一个旋转得到，求他们的旋转矩阵。
