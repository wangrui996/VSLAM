个人学习Vins_Mono源码的笔记  

视觉前端

* [1Vins公式推导](#1Vins公式推导)
  * [](#)
* [2Vins代码阅读笔记](#2Vins代码阅读笔记)
  * [视觉前端](#视觉前端)

## 1Vins公式推导

## 2Vins代码阅读笔记  

### 视觉前端  

前端主要工作包括：  
1.特征提取——获取特征点像素坐标  
2.特征点去畸变——得到去畸变后的归一化坐标  
3.特征点光流追踪  
4.特征点速度计算——用于相机和IMU时间戳矫正

vins_mono的前端作为了一个独立的ros节点

camera_model文件夹  
定义了Camera虚基类，针孔相机，鱼眼相机等派生类，具体相机的实例通过工厂模式生成。  
每个Camera虚基类内部都定义了一个参数Parameters类，也是个虚基类，在各派生类中也要实现它的纯虚函数






---

### Camera虚基类  

#### 成员函数  
#### 成员属性
##### public  

定义了枚举类型ModelType，并在protected中定义了它的一个变量m_modelType，被派生类继承后，通过yaml读取参数进来后赋值，作为相机类型  
###### Parameters虚基类
protected类型的成员
相机类型,相机内参,图像宽，高等所有派生类都会有的属性






---
### PinholeCamera针孔相机派生类  
该类公有继承自Camera类，内部Parameters类公有继承自Camera::Parameters
### 成员属性  
#### public  
公有继承了Camera::Parameters
protected类型的成员
除了继承自Camera的成员外，根据针孔相机特点又定义了部分参数，包括四个畸变参数(m_k1,m_k2,m_p1,m_p2)和内参(m_fx,m_fy,m_cx,m_cy)


### 成员函数   

#### setParameters
void PinholeCamera::setParameters(const PinholeCamera::Parameters& parameters)  
首先将parameters赋值给mParameters  
做一些预处理方便后面去畸变操作  

 
#### readFromYamlFile
bool PinholeCamera::Parameters::readFromYamlFile(const std::string& filename)  
根据配置文件读取相机参数，成功则返回true，失败返回false  
包括相机类型，相机名，相机畸变参数，内参










---
### feature_tracker_node.cpp  
#### 流程  
初始化ros节点，创建ros句柄  
读取配置文件   readParameters(n);  
[void readParameters(ros::NodeHandle &n)](#readparameters)  
feature追踪器(FeatureTracker类的对象，这里只有一个摄像头数组里面只有一个对象)通过成员函数获取内参  
    
    trackerData[i].readIntrinsicParameter(CAM_NAMES[i]);    // 获得每个相机的内参
[readIntrinsicParameter](#读内参)  
创建订阅者和发布者——对外发布三个话题，特征，图像和重启
ros::spin();循环，执行回调

#### 函数  

// 图片的回调函数  
void img_callback(const sensor_msgs::ImageConstPtr &img_msg)  
(1)判断是否是第一帧，是就记录时间，等待下一帧过来   
(2)检查时间戳是否正常，异常则执行reset(重新设置为还没有接收到第一帧前的状态)，并对其他模块发布重启话题  
(3)频率控制，控制发送给后端的频率    
首先计算当前频率(使用当前已发送次数除以当前时间与起始时间的间隔)，在预设频率范围内就发布当前帧，但是注意，这里计算的是一个平均频率      
虽然平均频率满足在预设范围内了，但是delta t越大，它对发送次数越不敏感，比如现在第100s，发送了900次，f=9，假如到第101s时发送了930次，此时计算f=9.3仍然满足f<10,但是从100s到101s的这段时间内的平均频率已经达到了30Hz，将导致后端压力过大；  
因此加一个判断：若当前这段时间的平均频率与预设频率很接近，就认为这段时间还比较符合要求，为了避免delta t太大，把初始时间和发布次数重置一下(恢复了)  
另外注意，即使不向后端发送,光流仍然进行(光流对连续两帧的时间要求比较高，太长时间间隔容易导致光流追踪失败)  
(4)通过cv_bridge转换ros图像为cv图像


---   
### feature_tracker.cpp feature_tracker.h

#### 成员属性  

```cpp
camodocal::CameraPtr m_camera; //相机实例  typedef boost::shared_ptr<Camera> CameraPtr；  
```

#### 成员函数
```cpp
void FeatureTracker::readIntrinsicParameter(const string &calib_file)
{
    ROS_INFO("reading paramerter of camera %s", calib_file.c_str());
    // 读到的相机内参赋给m_camera
    // camodocal::CameraPtr m_camera;  //? camodocal命名空间下定义了Camera虚基类 typedef boost::shared_ptr<Camera> CameraPtr;
    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}
```
传入配置文件路径，通过CameraFactory的静态成员函数创建实例[CameraFactory::instance()](#创建camerafactory实例)，紧接调用相机生成函数创建一个带有参数信息的相机实例generateCameraFromYamlFile(calib_file)(工厂模式)，[创建相机实例](#从yaml文件创建相机camera实例)  









##### 读内参


##### parameters.cpp  

##### 函数


#### readParameters

    void readParameters(ros::NodeHandle &n)


##### parameters.cpp  















---
### CameraFactory.cc  CameraFactory.h    
定义了CameraFactory类，

#### 函数  

#### 创建CameraFactory实例
```cpp
boost::shared_ptr<CameraFactory>
CameraFactory::instance(void)
{
    if (m_instance.get() == 0)
    {
        m_instance.reset(new CameraFactory);
    }

    return m_instance;
}
```

#### 从yaml文件创建相机Camera实例  
打开该yaml文件，如果打开不成功则返回一个不含指针的CameraPtr  
创建成功，根据yaml定义的相机类型创建具体派生类的实例,以针孔相机模型为例  PinholeCameraPtr camera(new PinholeCamera);  
有了实例接下来就是读取配置文件的相机参数到该实例，其中读取工作是由Parameters完成，因为camera实例中包含了一个Parameters型对象，因此下面通过公有接口先拿到这个对象，再调用读取函数  
通过PinholeCamera提供的公有函数getParameters()得到类的Parameters型私有成员并赋给params，这里函数返回的是引用。  

    params.readFromYamlFile(filename);
    camera->setParameters(params);  

[readFromYamlFile](#readfromyamlfile) 通过继承的Parameters类的接口实现  
注意，通过这一步读取参数，只是将参数放在了一个Parameters类的对象中了，这个
[setParameters](#setparameters) 通过PinholeCamera类的成员函数  
结束以后返回创建的这个camera  





























这是VINS-Mono开源代码的注释版本，方便大家学习这款非常优秀的VIO框架，原git库地址https://github.com/HKUST-Aerial-Robotics/VINS-Mono

关于代码的注释会不定期更新，欢迎关注

如果本仓库对你有用，欢迎star一下满足本人的虚荣心～

PS：Momenta 语义slam，多传感器融合，自动标定还有大量HC，有意向同学可以联系xieqi@momenta.ai

# VINS-Mono
## A Robust and Versatile Monocular Visual-Inertial State Estimator

**11 Jan 2019**: An extension of **VINS**, which supports stereo cameras / stereo cameras + IMU / mono camera + IMU, is published at [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)

**29 Dec 2017**: New features: Add map merge, pose graph reuse, online temporal calibration function, and support rolling shutter camera. Map reuse videos: 

<a href="https://www.youtube.com/embed/WDpH80nfZes" target="_blank"><img src="http://img.youtube.com/vi/WDpH80nfZes/0.jpg" 
alt="cla" width="240" height="180" border="10" /></a>
<a href="https://www.youtube.com/embed/eINyJHB34uU" target="_blank"><img src="http://img.youtube.com/vi/eINyJHB34uU/0.jpg" 
alt="icra" width="240" height="180" border="10" /></a>

VINS-Mono is a real-time SLAM framework for **Monocular Visual-Inertial Systems**. It uses an optimization-based sliding window formulation for providing high-accuracy visual-inertial odometry. It features efficient IMU pre-integration with bias correction, automatic estimator initialization, online extrinsic calibration, failure detection and recovery, loop detection, and global pose graph optimization, map merge, pose graph reuse, online temporal calibration, rolling shutter support. VINS-Mono is primarily designed for state estimation and feedback control of autonomous drones, but it is also capable of providing accurate localization for AR applications. This code runs on **Linux**, and is fully integrated with **ROS**. For **iOS** mobile implementation, please go to [VINS-Mobile](https://github.com/HKUST-Aerial-Robotics/VINS-Mobile).

**Authors:** [Tong Qin](http://www.qintonguav.com), [Peiliang Li](https://github.com/PeiliangLi), [Zhenfei Yang](https://github.com/dvorak0), and [Shaojie Shen](http://www.ece.ust.hk/ece.php/profile/facultydetail/eeshaojie) from the [HUKST Aerial Robotics Group](http://uav.ust.hk/)

**Videos:**

<a href="https://www.youtube.com/embed/mv_9snb_bKs" target="_blank"><img src="http://img.youtube.com/vi/mv_9snb_bKs/0.jpg" 
alt="euroc" width="240" height="180" border="10" /></a>
<a href="https://www.youtube.com/embed/g_wN0Nt0VAU" target="_blank"><img src="http://img.youtube.com/vi/g_wN0Nt0VAU/0.jpg" 
alt="indoor_outdoor" width="240" height="180" border="10" /></a>
<a href="https://www.youtube.com/embed/I4txdvGhT6I" target="_blank"><img src="http://img.youtube.com/vi/I4txdvGhT6I/0.jpg" 
alt="AR_demo" width="240" height="180" border="10" /></a>

EuRoC dataset;                  Indoor and outdoor performance;                         AR application;

<a href="https://www.youtube.com/embed/2zE84HqT0es" target="_blank"><img src="http://img.youtube.com/vi/2zE84HqT0es/0.jpg" 
alt="MAV platform" width="240" height="180" border="10" /></a>
<a href="https://www.youtube.com/embed/CI01qbPWlYY" target="_blank"><img src="http://img.youtube.com/vi/CI01qbPWlYY/0.jpg" 
alt="Mobile platform" width="240" height="180" border="10" /></a>

 MAV application;               Mobile implementation (Video link for mainland China friends: [Video1](http://www.bilibili.com/video/av10813254/) [Video2](http://www.bilibili.com/video/av10813205/) [Video3](http://www.bilibili.com/video/av10813089/) [Video4](http://www.bilibili.com/video/av10813325/) [Video5](http://www.bilibili.com/video/av10813030/))

**Related Papers**

* **Online Temporal Calibration for Monocular Visual-Inertial Systems**, Tong Qin, Shaojie Shen, IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS, 2018), **best student paper award** [pdf](https://ieeexplore.ieee.org/abstract/document/8593603)

* **VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator**, Tong Qin, Peiliang Li, Zhenfei Yang, Shaojie Shen, IEEE Transactions on Robotics[pdf](https://ieeexplore.ieee.org/document/8421746/?arnumber=8421746&source=authoralert) 

*If you use VINS-Mono for your academic research, please cite at least one of our related papers.*[bib](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/master/support_files/paper_bib.txt)

## 1. Prerequisites
1.1 **Ubuntu** and **ROS**
Ubuntu  16.04.
ROS Kinetic. [ROS Installation](http://wiki.ros.org/ROS/Installation)
additional ROS pacakge
```
    sudo apt-get install ros-YOUR_DISTRO-cv-bridge ros-YOUR_DISTRO-tf ros-YOUR_DISTRO-message-filters ros-YOUR_DISTRO-image-transport
```


1.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html), remember to **make install**.
(Our testing environment: Ubuntu 16.04, ROS Kinetic, OpenCV 3.3.1, Eigen 3.3.3) 

## 2. Build VINS-Mono on ROS
Clone the repository and catkin_make:
```
    cd ~/catkin_ws/src
    git clone https://github.com/HKUST-Aerial-Robotics/VINS-Mono.git
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```

## 3. Visual-Inertial Odometry and Pose Graph Reuse on Public datasets
Download [EuRoC MAV Dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets). Although it contains stereo cameras, we only use one camera. The system also works with [ETH-asl cla dataset](http://robotics.ethz.ch/~asl-datasets/maplab/multi_session_mapping_CLA/bags/). We take EuRoC as the example.

**3.1 visual-inertial odometry and loop closure**

3.1.1 Open three terminals, launch the vins_estimator , rviz and play the bag file respectively. Take MH_01 for example
```
    roslaunch vins_estimator euroc.launch 
    roslaunch vins_estimator vins_rviz.launch
    rosbag play YOUR_PATH_TO_DATASET/MH_01_easy.bag 
```
(If you fail to open vins_rviz.launch, just open an empty rviz, then load the config file: file -> Open Config-> YOUR_VINS_FOLDER/config/vins_rviz_config.rviz)

3.1.2 (Optional) Visualize ground truth. We write a naive benchmark publisher to help you visualize the ground truth. It uses a naive strategy to align VINS with ground truth. Just for visualization. not for quantitative comparison on academic publications.
```
    roslaunch benchmark_publisher publish.launch  sequence_name:=MH_05_difficult
```
 (Green line is VINS result, red line is ground truth). 
 
3.1.3 (Optional) You can even run EuRoC **without extrinsic parameters** between camera and IMU. We will calibrate them online. Replace the first command with:
```
    roslaunch vins_estimator euroc_no_extrinsic_param.launch
```
**No extrinsic parameters** in that config file.  Waiting a few seconds for initial calibration. Sometimes you cannot feel any difference as the calibration is done quickly.

**3.2 map merge**

After playing MH_01 bag, you can continue playing MH_02 bag, MH_03 bag ... The system will merge them according to the loop closure.

**3.3 map reuse**

3.3.1 map save

Set the **pose_graph_save_path** in the config file (YOUR_VINS_FOLEDER/config/euroc/euroc_config.yaml). After playing MH_01 bag, input **s** in vins_estimator terminal, then **enter**. The current pose graph will be saved. 

3.3.2 map load

Set the **load_previous_pose_graph** to 1 before doing 3.1.1. The system will load previous pose graph from **pose_graph_save_path**. Then you can play MH_02 bag. New sequence will be aligned to the previous pose graph.

## 4. AR Demo
4.1 Download the [bag file](https://www.dropbox.com/s/s29oygyhwmllw9k/ar_box.bag?dl=0), which is collected from HKUST Robotic Institute. For friends in mainland China, download from [bag file]().

4.2 Open three terminals, launch the ar_demo, rviz and play the bag file respectively.
```
    roslaunch ar_demo 3dm_bag.launch
    roslaunch ar_demo ar_rviz.launch
    rosbag play YOUR_PATH_TO_DATASET/ar_box.bag 
```
We put one 0.8m x 0.8m x 0.8m virtual box in front of your view. 

## 5. Run with your device 

Suppose you are familiar with ROS and you can get a camera and an IMU with raw metric measurements in ROS topic, you can follow these steps to set up your device. For beginners, we highly recommend you to first try out [VINS-Mobile](https://github.com/HKUST-Aerial-Robotics/VINS-Mobile) if you have iOS devices since you don't need to set up anything.

5.1 Change to your topic name in the config file. The image should exceed 20Hz and IMU should exceed 100Hz. Both image and IMU should have the accurate time stamp. IMU should contain absolute acceleration values including gravity.

5.2 Camera calibration:

We support the [pinhole model](http://docs.opencv.org/2.4.8/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html) and the [MEI model](http://www.robots.ox.ac.uk/~cmei/articles/single_viewpoint_calib_mei_07.pdf). You can calibrate your camera with any tools you like. Just write the parameters in the config file in the right format. If you use rolling shutter camera, please carefully calibrate your camera, making sure the reprojection error is less than 0.5 pixel.

5.3 **Camera-Imu extrinsic parameters**:

If you have seen the config files for EuRoC and AR demos, you can find that we can estimate and refine them online. If you familiar with transformation, you can figure out the rotation and position by your eyes or via hand measurements. Then write these values into config as the initial guess. Our estimator will refine extrinsic parameters online. If you don't know anything about the camera-IMU transformation, just ignore the extrinsic parameters and set the **estimate_extrinsic** to **2**, and rotate your device set at the beginning for a few seconds. When the system works successfully, we will save the calibration result. you can use these result as initial values for next time. An example of how to set the extrinsic parameters is in[extrinsic_parameter_example](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/master/config/extrinsic_parameter_example.pdf)

5.4 **Temporal calibration**:
Most self-made visual-inertial sensor sets are unsynchronized. You can set **estimate_td** to 1 to online estimate the time offset between your camera and IMU.  

5.5 **Rolling shutter**:
For rolling shutter camera (carefully calibrated, reprojection error under 0.5 pixel), set **rolling_shutter** to 1. Also, you should set rolling shutter readout time **rolling_shutter_tr**, which is from sensor datasheet(usually 0-0.05s, not exposure time). Don't try web camera, the web camera is so awful.

5.6 Other parameter settings: Details are included in the config file.

5.7 Performance on different devices: 

(global shutter camera + synchronized high-end IMU, e.g. VI-Sensor) > (global shutter camera + synchronized low-end IMU) > (global camera + unsync high frequency IMU) > (global camera + unsync low frequency IMU) > (rolling camera + unsync low frequency IMU). 

## 6. Docker Support

To further facilitate the building process, we add docker in our code. Docker environment is like a sandbox, thus makes our code environment-independent. To run with docker, first make sure [ros](http://wiki.ros.org/ROS/Installation) and [docker](https://docs.docker.com/install/linux/docker-ce/ubuntu/) are installed on your machine. Then add your account to `docker` group by `sudo usermod -aG docker $YOUR_USER_NAME`. **Relaunch the terminal or logout and re-login if you get `Permission denied` error**, type:
```
cd ~/catkin_ws/src/VINS-Mono/docker
make build
./run.sh LAUNCH_FILE_NAME   # ./run.sh euroc.launch
```
Note that the docker building process may take a while depends on your network and machine. After VINS-Mono successfully started, open another terminal and play your bag file, then you should be able to see the result. If you need modify the code, simply run `./run.sh LAUNCH_FILE_NAME` after your changes.


## 7. Acknowledgements
We use [ceres solver](http://ceres-solver.org/) for non-linear optimization and [DBoW2](https://github.com/dorian3d/DBoW2) for loop detection, and a generic [camera model](https://github.com/hengli/camodocal).

## 8. Licence
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.

We are still working on improving the code reliability. For any technical issues, please contact Tong QIN <tong.qinATconnect.ust.hk> or Peiliang LI <pliapATconnect.ust.hk>.

For commercial inquiries, please contact Shaojie SHEN <eeshaojieATust.hk>
