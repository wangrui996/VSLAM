# vins_fusion与vins_mono区别点  


## 前端部分  

### vins_mono

两者都不是把每帧光流结果都发布给后端，而是控制了发布频率  

* vins_mono前后端采用不同ROS节点，也就是不同的进程，前段光流处理结果(包括当前帧特征点坐标，速度等数据)，封装成ros消息sensor_msgs::PointCloud发布给后端  
* vins_mono发布给后端的特征点被追踪的次数大于1才发送，不然无法构成重投影约束，也没法三角化  


### vins-fusion

**vins-fusion前端和后端都在一个进程中，采用了多线程的写法**  

* estimator后端源文件调用了feature_tracker的接口来对图像进行光流追踪，前端处理光流追踪结束后，将特征点信息放在一个map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>类型的变量featureFrame中返回；
* 在estimator获取featureFrame后，与当前图像时间(t),构成pair并加入特征队列featureBuf  然后根据MULTIPLE_THREAD决定是否执行processMeasurements()函数
* 在estimator_node节点中，也订阅了/feature_tracker/feature话题，但是看代码这里似乎是为了和vins_mono统一起来但实际并没有用到？，因为没有节点会发布这个话题；
    * 同时，查看这个话题的回调函数可知，它将sensor_msgs::PointCloud的ros消息也是拆分成并重新封装成了featureFrame，然后根据MULTIPLE_THREAD决定是否执行processMeasurements()函数  
* 可以看到，这里如果MULTIPLE_THREAD参数为0，则获取到特征队列featureBuf以后会调用processMeasurements()处理测量数据     如果为1，没有调用processMeasurements()  




#### MULTIPLE_THREAD参数  

如果MULTIPLE_THREAD参数为0，则获取上述featureBuf后，调用processMeasurements()处理测量数据；  


#### vins-fusion中的processMeasurements()函数  

