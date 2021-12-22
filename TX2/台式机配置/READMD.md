# 台式机配置  

## V2Ray科学上网  

https://mahongfei.com/1776.html
https://github.com/Qv2ray/Qv2ray/releases
https://github.com/v2ray/v2ray-core/releases/

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





