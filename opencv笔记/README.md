
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
(1) 从文件读取参数    
使用重载的运算符[]加操作符>>  
fs["文件中变量的名"] >> cpp文件中变量的名字
```cpp
cv::FileStorage fs(config_path, cv::FileStorage::READ);

//C++基本数据类型
string path;
int kp;
fs["PATH"] >> path;
kp = fs["KP"];

//数组
vector<int> v1;
fs["vector_Kp"] >> v1;
for(int i = 0; i < v1.size(); i++)
{
    cout << v1[i] << " ";
}
cout << endl;

//opencv类型，常用Mat型
cv::Mat K;
fs["Cam_K"] >> K;
cout << K << endl;
```  
(2) 写入文件  
使用操作符<<   fs << "文件中变量的名" << 变量的值;
```cpp
cv::FileStorage fs(config_path, cv::FileStorage::WRITE);
//字符串
string path = "/home";  
fs << "PATH" << path; //将path写入文件中，文件中名称为PATH
//数组
vector<int> v2(1, 2, 3);
fs << "vector_v2" << v2;
```  
也可以进行其它数据类型的读写操作，如SEQ(sequence)、MAP(mapping),[参考](https://blog.csdn.net/sandalphon4869/article/details/104020330?utm_medium=distribute.pc_relevant.none-task-blog-2~default~baidujs_title~default-0.control&spm=1001.2101.3001.4242)


#### 3、关闭
virtual void cv::FileStorage::release()  
关闭文件并释放所有内存缓冲区,在对文件操作完成后调用此方法。  


