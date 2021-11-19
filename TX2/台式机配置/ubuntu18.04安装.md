# ubuntu18.04安装.md  

## 制作U盘系统工具  

准备： 金士顿64G U盘  

[安装盘制作工具refus下载](https://rufus.ie/zh/#google_vignette)  

分区类型：MBR  目标系统类型：BIOS或UEFI
文件系统：large FAT32 默认  簇大小 32K默认大小

分区：  
    * ext4  /boot 5G  
    * 交换空间 16G  
    * ext4 /home 600G
    * ext4 /     剩余  

## 关于双系统安装方式  
需要注意的是： bios中的Legacy引导方式只能配合MBR的磁盘格式，uefi可以和GPT磁盘格式配合（UEFI与MBR没有试过）  

win10 采用U深度，进入后用磁盘管理工具将固态格式化成MBR格式，安装win10  
ubuntu 采用refus制作，ubuntu18.04.1版本，MBR格式  

由于MBR磁盘格式需要传统引导方式，BIOS中启动设置，最后三个均设置为仅Legacy  

ubuntu安装分区好后，引导位置安装在默认的地方（西数蓝盘固态），而不需要安装在/boot下，这样，最后是通过ubuntu引导windows  

如果grub不能引导windows，可以在ubunut下执行sudo update-grub(udo update-grub2)更新下  

由于启动界面还加载了另外两个不需要的东西，windows的名字也是windows vista而不是win10，所以我先备份了一个/boot/grub/grub.cfg文件，之后对它进行了修改  
注释掉了里面不需要的东西  




