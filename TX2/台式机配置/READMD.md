# 台式机配置  

## V2Ray科学上网  

https://mahongfei.com/1776.html
https://github.com/Qv2ray/Qv2ray/releases
https://github.com/v2ray/v2ray-core/releases/

##  Ubuntu终端配置代理
用下面的方式下载ProxyChains 
[ProxyChains](https://www.cnblogs.com/guguobao/p/8878109.html)
用下面方式配置  
[proxychains配置](https://docs.shanyuhai.top/os/manjaro/terminal-agent.html#proxychains-ng)




## 显卡驱动  
libnvidia-cfg1-460:amd64                     460.56-0ubuntu0.18.04.1 


#
[conda创建虚拟环境安装cuda，cudnn，tensort](https://blog.csdn.net/weixin_41010198/article/details/107604593?utm_medium=distribute.pc_relevant.none-task-blog-2~default~baidujs_title~default-1.no_search_link&spm=1001.2101.3001.4242.2)


## CUDA/cuDNN  

cuda版本：10.1  
cuDNN版本：7.6.5

## CUDA切换10.0与10.1版本  

cuda-10.0切换后才能cuda-10.1  
* 1.sudo rm -rf /usr/local/cuda #删除之前生成的软链接  
* 2.sudo ln -s /usr/local/cuda-10.1 /usr/local/cuda #生成新的软链接  


## pytorch环境配置  

Python 3.6.3 |Anaconda, Inc.| (default, Oct 13 2017, 12:02:49)
Anaconda

## 查看conda版本  
wangrui@wangrui:~$ conda -V
conda 4.9.2


Anaconda3-5.1.0-Windows-x86_64.exe     对应 python3.6.3
[清华大学开源软件镜像站](https://mirrors.tuna.tsinghua.edu.cn/anaconda/archive/)

python版本：3.6.3

##   TensorRT  
TensorRT下载    
[TensorRT Download](https://developer.nvidia.com/nvidia-tensorrt-download)    

**台式机使用：**  
TensorRT 7.0.0 配合CUDA10.0  cuDNN7.6.5    

### 使用TensorRT遇到的问题  
 * **问题1**  
报错找不到 NvInfer.h   
**解决：**  
因为我的tensorRT是下载tar解压安装的，库文件和头文件都在下载解压的目录，因此需要修改CMakeLists.txt，设置其头文件和库文件的路径  

```txt  
# tensorrt  
include_directories(/home/wangrui/下载/TensorRT/TensorRT-7.0.0.11/include)  
link_directories(/home/wangrui/下载/TensorRT/TensorRT-7.0.0.11/lib)  
```  


## 记录opencv问题  
* 1. 希望使用自定义版本的opencv,台式机使用了opencv3.4.1， 因此更换了cv_bridge版本，具体见TX2中源码安装cv_bridge部分  
* 2. 在编译vins和orb_slam时：
    * 在find cv_bridge的地方设置了set(cv_bridge_DIR /usr/local/cv_bridge341/share/cv_bridge/cmake) 
    * 在find opencv的地方设置了opencv版本为3.4.1（或者set  opencv的cmake文件位置)  
最后在运行时仍然发现问题：
  大致报错是load library (Poco exception = libopencv_core.so.3.2: cannot open shared object file: No such file or directory)
由于已经将3.2版本的opencv库文件删除了所以提示找不到是正常的（许多ros默认安装的包都还是默认使用opencv3.2版本），但为什么我明明在CMakeLists中设置了cv_bridge的路径和opencv版本，最后链接的还是opencv3.2？   
**解决方案：** 当时解决方案没有及时记录下来，但大概是因为ROS工作空间的原因，在编译vins-fusion-gpu和ORB_SLAM3时遇到的问题不太一样  
  * VINS-fusion-gpu版本：
  * ORB_SLAM3的ROS包中CMakeLists文件中没有find cv_bridge，最后虽然设置了opencv的版本是3.4.1，但是里面用到了cv_bridge还是自带的包，使用的还是opencv3.2版本，最后提示找不到opencv库，最后解决方案是显示设置cv_bridge包，把库链接到可执行文件上
  set(cv_bridge_DIR "/usr/local/cv_bridge341/share/cv_bridge/cmake")  
  find_package(cv_bridge REQUIRED)
  link_directories("/usr/local/cv_bridge341/lib")
  target_link_libraries(Mono
  ${LIBS}
  cv_bridge
  )

## usb_cam  

我源码安装usb_cam时也出现了类似上面这个问题，启动后发现中找不到opencv3.2的库，看了下usb_cam的CMakeLists文件，usb_cam本身没问题没用到opencv，实际上是image_view这个包报的错；可以源码安装image_view或者在usb_cam的launch文件中注释调image_view这个节点  




