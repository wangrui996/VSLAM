## ROS笔记  

# ROS基础  
* [1基础](#1基础)
  * [ROS命名空间](#ROS命名空间)
  * [ROS参数服务器](#ROS参数服务器)

* [相关公司](#相关公司)



## 1基础  

## ROS命名空间
假设launch 文件中 ns=="namespace"
```cpp
ros::init(argc, argv, "node_name"); //节点名称为  node_name
 
ros::NodeHandle n; //n 命名空间为 /namespace  括号中代表launch文件中ns未指定的情况(/)

ros::NodeHandle nh1("~"); //nh1 命名空间为/namespace/node_name (/node_name)
ros::NodeHandle nh2("~vins"); //nh2 命名空间为/namespace/node_name/vins (/node_name/vins)
ros::NodeHandle nh3("~/vins"); //nh3 命名空间为/namespace/node_name/vins (/node_name/vins)

ros::NodeHandle nh4("control");  //nh4命名空间为/namespace/control (/control)
ros::NodeHandle nh5(nh4,"pid");  //nh5命名空间为/namespace/control/pid (/control/pid)

ros::NodeHandle gn("/global"); // gn 命名空间为/global
```
如果launch文件没有指定ns，则相应的/namespace换成默认的 /



### ROS参数服务器  
根据ROS WiKi，rosparam允许在ROS参数服务器上存储和操作数据，包括整型，浮点型，布尔型等等，rosparam使用YAML标记语言，使用rosparam -h命令，可以查看支持的命令如下：
```bash
rosparam set            set parameter
rosparam get            get parameter
rosparam load           load parameters from file
rosparam dump           dump parameters to file
rosparam delete         delete parameter
rosparam list           list parameter names
```

使用方法：  
1.使用roslaunch  
在launch文件中配置好参数的初始值，可以在程序中获取后进一步使用，想要修改参数的初始值时不需要修改和编译源码。
(1)使用<param>标签定义参数  
  <param name="PID_K" type="double" value="4.0" />
(2)利用<rosparam>标签从YAML文件中读取参数    

```YAML
<launch>
  <node pkg="controller" type="px4_pose_controller" name="px4_pose_controller" output="screen">
     <rosparam file="$(find controller)/config/px4_pose_controller.yaml"/>
  </node>		
</launch>
```  
另外，也可以通过<rosparam>标签直接对某个参数赋值，或者删除某个参数  
2.通过命令行操作参数  
利用rosparam -h提示的命令可以查看已存在的参数，设置，获取某个参数，或者从文件加载参数等  
3.通过cpp文件  
一般我们在写ros包时，都会把roscpp加入依赖，它为我们提供了基本的cpp和ros的接口，包括参数相关接口。  
有两套API可供我们使用，一是ros::NodeHandle提供的接口，二是ros::param命名空间下的一些接口。    

为参数设置默认值 
n.param<变量类型>("参数名", 变量名， 默认值);
ros::param::param<变量类型>("参数名", 变量名， 默认值);

设置参数  
n.setParam("参数名"， value);
ros::param::set("参数名"， value);  

读取参数  
n.getParam("参数名"， 变量名);
ros::param::get("参数名"， 变量名);
  
```cpp
ros::NodeHandle n; //全局
ros::NodeHandle nh("~"); //波浪线表示句柄所在命名空间为该节点

//为参数设置默认值
n.param<type>
  
```  























