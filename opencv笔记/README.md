
## opencv笔记  

### [opencv3.4.1官方文档](https://docs.opencv.org/3.4.1/d2/d75/namespacecv.html)

## opencv  
* [1基础](#1基础)
  * [读写yaml文件](#读写yaml文件)
  * [ROS参数服务器](#ROS参数服务器)

* [相关公司](#相关公司)  



### 读写yaml文件  
opencv提供了cv::FileStorage类(文件存储)，支持对XML/YAML/JSON格式的文件进行读写操作,查看该类的定义，首先使用枚举类型定义了文件存储的模式，常用的有WRITE，READ等  

[FileStorage官方文档](https://docs.opencv.org/3.4.1/da/d56/classcv_1_1FileStorage.html)

使用时需要注意，类似于C++中使用文件流操作文件，使用时代码中需要利用提供的接口判断文件是否被正常打开，一个FileStorage句柄只能在对一个文件操作完成，并且关闭后才能再进行其他操作，
也不能有两个FileStorage句柄同时读或写(一个关闭以后另外一个才能进行)。下面结合常用的操作进行说明  
#### 1.打开文件  
a、通过一个重载的带参构造函数打开一个文件  
```cpp
cv::FileStorage::FileStorage (const String & filename,
int 	flags,
const String & 	encoding = String() 
)	
```
filename: 文件名(.xml, .yml/.yaml or .json)  
flags: 操作模式，包括FileStorage::READ读取，FileStorage::WRITE写入，FileStorage::APPEND追加等  
encoding：文件的编码格式，支持UTF-8，这个参数不需要管  
```cpp
string config_path = "/home/wr/config/vins.yaml";
cv::FileStorage fsSettings(config_path, cv::FileStorage::READ);
```
b、使用无参构造函数，然后调用FileStorage::open()打开一个文件,open函数的参数列表与上面的构造函数是一致的
```cpp
string config_path = "/home/wr/config/vins.yaml";
cv::FileStorage fsSettings;
fsSettings.open(config_path, cv::FileStorage::READ);
```

#### 2、判断是否打开成功isOpened()  
virtual bool cv::FileStorage::isOpened	(		)	const  
成功返回true、失败返回false
```cpp
cv::FileStorage fs(config_path, cv::FileStorage::READ);
if (!fs.isOpened())
{
    std::cerr << "failed to open " << config_path << std::endl;
    return 1;
}
```  

#### 读写操作  
读写操作可以使用FileStorage的成员函数，也可以使用其他opencv函数(需要传入FileStorage对象)
(1) 读参数  
使用重载的运算符[]  
```cpp
cv::FileStorage fs(config_path, cv::FileStorage::READ);
string path;
int kp;
fs["PATH"] >> path;
kp = fs["KP"];
```



#### 3、关闭
virtual void cv::FileStorage::release()  
关闭文件并释放所有内存缓冲区,在对文件操作完成后调用此方法。  


