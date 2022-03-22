# opencv4.5.5安装过程  

台式机环境：nvidia 460.91.03 驱动  cuda 10.0  cudnn 7.6.5



1. 下载opencv4.5.5 和opencv_contrib-4.5.5  
2. 将opencv_contrib-4.5.5解压在opencv4.5.5目录下  

3.编译带有gpu的opencv4.5.5  

由于之前opencv3.4.1安装到了/usr/local下  为了管理起来方便这里安装在/usr/local/opencv455 

CUDA_ARCH_BIN 对应的是7.5

```shell
cmake -D CMAKE_BUILD_TYPE=RELEASE -D INSTALL_PYTHON_EXAMPLES=OFF -DCMAKE_INSTALL_PREFIX=/usr/local/opencv455 -D INSTALL_C_EXAMPLES=OFF  -D OPENCV_EXTRA_MODULES_PATH=/home/wr/Tools/opencv-4.5.5/opencv_contrib-4.5.5/modules -D WITH_TBB=ON  -D WITH_V4L=ON -D WITH_QT=ON  -D WITH_GTK=ON  -D BUILD_EXAMPLES=ON  -D WITH_CUDA=ON -D CUDA_ARCH_BIN="7.5" -D CUDA_ARCH_PTX=""  -D WITH_OPENGL=ON -D WITH_GTK=ON ..
```

编译  
```shell
make -j16
```
