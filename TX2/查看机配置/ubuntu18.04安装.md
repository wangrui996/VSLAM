# ubuntu18.04安装.md  

## 制作U盘系统工具  

准备： 金士顿64G U盘  

[安装盘制作工具refus下载](https://rufus.ie/zh/#google_vignette)  

分区类型：MBR  目标系统类型：BIOS或UEFI
文件系统：large FAT32 默认  簇大小 32K默认大小

分区：  
    * ext4  /boot 5G  
    * 交换空间 16G  
    * ext4 /home 500G
    * ext4 /     200
    * ext4 /usr  200   
    
**系统恢复或重装注意事项**  
保留/home和/usr目录下已有数据，只需要在重装时点击修改/home，然后重新选择挂载点还是/home，不要选择格式化；如下图；同理/usr也是，这样装完后，新系统自带的文件会覆盖/usr下原有的文件，但我们自己编译安装到/usr中的头文件库文件等还会存在；    


## 关于双系统安装方式  
需要注意的是： bios中的Legacy引导方式只能配合MBR的磁盘格式，uefi可以和GPT磁盘格式配合（UEFI与MBR没有试过）  

win10 采用U深度，进入后用磁盘管理工具将固态格式化成MBR格式，安装win10  
ubuntu 采用refus制作，ubuntu18.04.1版本，MBR格式  

由于MBR磁盘格式需要传统引导方式，BIOS中启动设置，最后三个均设置为仅Legacy  

ubuntu安装分区好后，引导位置安装在默认的地方（西数蓝盘固态），而不需要安装在/boot下，这样，最后是通过ubuntu引导windows  

如果grub不能引导windows，可以在ubunut下执行sudo update-grub(udo update-grub2)更新下  

由于启动界面还加载了另外两个不需要的东西，windows的名字也是windows vista而不是win10，所以我先备份了一个/boot/grub/grub.cfg文件，之后对它进行了修改  
注释掉了里面不需要的东西  

## ubuntu新系统配置  
1.[关闭内核自动更新](https://blog.csdn.net/weixin_45629790/article/details/112538569)

2.更换软件源  

3.安装有限网卡驱动  
e1000e  
BIOS设置：快速启动 关闭    密钥：全删除  

4.nvida驱动安装  

4.1 首先禁用nouveau  
在/etc/modprobe.d/blacklist.conf里添加

      blacklist nouveau
      options nouveau modeset=0  

查询可用的nvidia版本：ubuntu-drivers devices  

因为后面要用cuda10.0和cudnn7.6.5，安装nvidia-drivers-460后执行nvidia-smi发现CUDA版本是11.2  


[英伟达驱动安装方式cuda10.0 cudnn7.6.5](https://www.cxyzjd.com/article/qq_39462585/111991678)  


4.2 cuda10.0安装  
[cuda10.0下载](https://developer.nvidia.com/cuda-toolkit-archive)  

[安装过程参考](https://www.cxyzjd.com/article/qq_39462585/111991678)  

选择了版本：nvidia-drivers-460  


## TX2备份与恢复  




## d455驱动  
针对台式机：  


  





