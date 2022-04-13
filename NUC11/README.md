
#

1.安装adimax无线网卡驱动
编译时报错 <stdarg.h>这个头文件找到   

查找它的位置然后创建软链接  sudo ln -s ...../stdarg.h  /lib/modules/4.15.0-29-generic/build/include/stdarg.h
