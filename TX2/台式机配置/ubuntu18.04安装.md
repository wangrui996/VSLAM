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
