# 线性代数基础知识  




## 矩阵的逆  

**定义：** 如果一个n阶方阵A，存在一个n阶方阵B使得AB = BA = E，E为单位阵，称A是可逆的，B是A的逆，记作A^(-1)  

**矩阵可逆的充要条件：**   


### 可逆矩阵的一些性质：  

* 1.n阶可逆矩阵A乘一个n维非零向量x，结果是一个n维的非零向量Ax  
证明： 反证法：假设Ax = 0，等式两端左乘一个A^(-1)，得到x = 0，矛盾  


## 对称矩阵  

一个矩阵的转置等于它自身，则称A为对称矩阵  
<p align="center"><img src="https://user-images.githubusercontent.com/58176267/146130840-48d4feff-9a9b-417f-bf83-37ba8cf57b3e.png"></p>



<p align="center"><img src="https://user-images.githubusercontent.com/58176267/146130800-d9eb840b-bc42-4dcf-bb18-490347582ecd.png"></p>




## 正定矩阵  

**定义：** 给定一个大小为 n x n 的实对称矩阵 A，若对于任意长度为n 的非零向量 x ，有 :  

<p align="center"><img src="https://user-images.githubusercontent.com/58176267/146134734-3e6ac855-1e6e-4483-8208-21594e25867f.png"></p>  
恒成立，则矩阵 A 是一个正定矩阵。

注意正定半正定矩阵都是定义在实对称矩阵上的  



## 矩阵的行变换和列变换  










