# Pangolin  
许多SLAM系统的可视化都是采用Pangolin可视化库做的  

## 官方文档  

[Pangolin官方文档](http://docs.ros.org/en/fuerte/api/pangolin_wrapper/html/namespacepangolin.html)  

[Pangolin博客](https://blog.csdn.net/weixin_43991178/article/details/105119610)  

[ORB_SLAM2中使用pangolin画相机](https://www.codenong.com/cs108921185/)

## OpenGL相关  
[OpenGL学习](https://www.cnblogs.com/Anita9002/p/4386472.html)

[glBlendFunc()函数——设置颜色混合 透明度叠加计算](http://blog.chinaunix.net/uid-20622737-id-2850251.html)  

### 开启深度测试  
当场景中出现一个物体遮挡另一个物体时，为了看清楚到底谁遮挡了谁，需要启动深度检测。


## 常用操作  

### 1.创建窗体  


### 2.创建观察相机视图  结构体pangolin::OpenGlRenderState


pangolin::OpenGlRenderState s_cam();

前面创建窗体的基础上，在视窗中“放置”一个观察相机，需要给出该相机的**内参矩阵**、初始时刻位置、相机视点位置（光轴朝向哪一个点）、相机哪一轴朝上  
[pangolin::OpenGlRenderState结构体说明](http://docs.ros.org/en/fuerte/api/pangolin_wrapper/html/structpangolin_1_1OpenGlRenderState.html)  
可以通过OpenGlRenderState (const OpenGlMatrix &projection_matrix, const OpenGlMatrix &modelview_matrix)构造一个“观察相机”，需要两个参数：  

**定义相机投影模型**   
下面是代码，几个参数分别代表：宽度、高度、4个内参、最近视距、最远视距
```cpp
// Use OpenGl's default frame of reference
OpenGlMatrixSpec ProjectionMatrix(int w, int h, double fu, double fv, double u0, double v0, double zNear, double zFar )
{
    return ProjectionMatrixRUB_BottomLeft(w,h,fu,fv,u0,v0,zNear,zFar);
}
```  
根据函数内部调用的ProjectionMatrixRUB_BottomLeft(w,h,fu,fv,u0,v0,zNear,zFar)可知：  
    Camera Axis:
    X - Right, Y - Up, Z - Back  

**定义观测方位向量**  
函数原型：
OpenGlMatrix pangolin::ModelViewLookAt	(double ex,
double 	ey,
double 	ez,
double 	lx,
double 	ly,
double 	lz,
double 	ux,
double 	uy,
double 	uz 
)	 
参数分别代表：
* 观测点位置：ex ey ez
* 观测目标位置：lx ly lz
* 观测的方位向量：ux uy uz  

### 3.定义显示视图（显示面板）  
**定义显示视图句柄**  
    pangolin::Handler3D handler(s_cam); //显示视图句柄，参数：要用一个相机视图

这里定义显示视图的目的是显示2中相机"拍摄"到的内容,  
函数原型：
    View & pangolin::CreateDisplay()  

返回一个结构体类型View的引用，[关于结构体View的继承关系与成员函数](http://docs.ros.org/en/fuerte/api/pangolin_wrapper/html/structpangolin_1_1View.html)  


* 通过SetBounds()函数设置显示视图在视窗中的范围（下、上、左、右）  
函数原型：
```cpp  
    View & pangolin::View::SetBounds(Attach bottom,
    Attach 	top,
    Attach 	left,
    Attach 	right,
    double 	aspect 
    )
```
最后一个参数表示长宽比：  

这里有两种设置方法：      
    * 1.可以使用相对坐标（0~1），即0表示
    * 2.  

* 通过SetHandler()
函数原型：  
    View & pangolin::View::SetHandler(Handler * handler)	

```cpp
pangolin::View& d_cam = pangolin::CreateDisplay()
    .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f/768.0f)
    .SetHandler(new pangolin::Header3D(s_cam));
```


### 清除缓冲区缓存  

* 1.清除缓冲区颜色和深度缓存
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);  


