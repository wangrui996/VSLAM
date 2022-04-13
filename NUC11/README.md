
#

1.安装adimax无线网卡驱动
编译时报错 <stdarg.h>这个头文件找到   

查找它的位置然后创建软链接  sudo ln -s ...../stdarg.h  /lib/modules/4.15.0-29-generic/build/include/stdarg.h


2.禁用内核更新  

1.sudo dpkg -l | grep linux 查看已经安装的内核  

2.保持锁定内核

```shell
sudo apt-mark hold linux-image-4.15.0-29-generic
sudo apt-mark hold linux-modules-extra-4.15.0-29-generic
```

3.重启内核更新

```shell
sudo apt-mark unhold linux-image-4.15.0-29-generic
sudo apt-mark unhold linux-modules-extra-4.15.0-29-generic
```

3.systemback备份  

[systemback安装](https://blog.csdn.net/rechardchen123/article/details/90649208)  
