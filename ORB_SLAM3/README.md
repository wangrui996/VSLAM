#  ORB_SLAM3  

## 编译安装  

1，报错：
```cpp
error: no match for ‘operator/’ (operand types are ‘cv::Matx<float, 3, 1>’ and ‘float’)
                 x3D = x3D_h.get_minor<3,1>(0,0) / x3D_h(3);
```   
替换为：
```cpp
x3D = cv::Matx31f(x3D_h.get_minor<3,1>(0,0)(0) / x3D_h(3), x3D_h.get_minor<3,1>(0,0)(1) / x3D_h(3), x3D_h.get_minor<3,1>(0,0)(2) / x3D_h(3));
```
