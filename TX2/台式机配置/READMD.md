# 台式机配置  


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

