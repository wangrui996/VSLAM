# VSLAM
SLAM学习过程记录
* [1SLAM基础知识](#1SLAM基础知识)
  * [](#)
  * [基础矩阵F本质矩阵E单应矩阵H](#基础矩阵F本质矩阵E单应矩阵H)
  * [三角化](#三角化)
  * [1-1卡方检验](#1-1卡方检验)

* [视觉词袋Bow](#视觉词袋Bow)
* [相关公司](#相关公司)
 
 估计单应性的DLT法 https://github.com/dusty-nv/homography-dlt
 
## 1SLAM基础知识

### 基础矩阵F本质矩阵E单应矩阵H

首先根据十四讲中作如下定义  
1.基本变量定义  
设空间点P在第一帧图像的相机坐标系下坐标：
<p align="center"><img src="https://user-images.githubusercontent.com/58176267/129569543-7e2c370b-e4fd-4411-bd23-f07365b89b2f.png"></p>  
相机内参为K  
<p align="center"><img src="https://user-images.githubusercontent.com/58176267/129570065-885775d2-39de-41ce-861f-e5341d6910f5.png"></p>  
P点在第一帧相机坐标系下深度为s1, 第二帧相机坐标系下深度为s2, R，t为相机坐标1到相机坐标系2的变换

2.本质矩阵E与基础矩阵F：  

<p align="center"><img src="https://user-images.githubusercontent.com/58176267/129570421-d0f20616-dd64-424e-90b6-b683ea3e1456.png"></p>  
<p align="center"><img src="https://user-images.githubusercontent.com/58176267/129570541-e7b84d92-b40d-4d02-a4b1-f0de71815000.png"></p>  
等式两边同时除以K的逆  
<p align="center"><img src="https://user-images.githubusercontent.com/58176267/129571421-f53bd631-4781-44b6-8bfc-801cc59d5e0b.png"></p>  
<p align="center"><img src="https://user-images.githubusercontent.com/58176267/129571360-15fcea6f-472d-4ace-9632-144a2bbabf5f.png"></p>  
记x1,x2分别为两像素点的归一化平面上的坐标（相当于把P分别除以各种坐标系下的深度s1或s2，得到的坐标，本质上还是在相机坐标系的坐标）  
<p align="center"><img src="https://user-images.githubusercontent.com/58176267/129571827-04f6c4f0-42b1-4443-81ca-9b873bf8af1c.png"></p>  
<p align="center"><img src="https://user-images.githubusercontent.com/58176267/129571951-3754f82f-74fa-4a90-a270-7c626802dfee.png"></p>  
好的最后这个式子右边是个加法，不太简洁，我们左右同乘t的反对称矩阵即可消去t(可以看下反对称矩阵的概念，左乘t的反对称矩阵相当于和t做叉乘，t和自己做叉乘为0向量)  

<p align="center"><img src="https://user-images.githubusercontent.com/58176267/129572767-eea88853-a871-4d24-9c21-86d5cc9d6afa.png"></p>  
这个时候还不太简洁，要是一边能等于0就好了，左边s2是个常数，剩下的是个与t和x2都垂直的向量，所以我们两边同时乘以x2的转置，左边就等于0，为什么乘x2的专置不去乘t的专置？因为那样右边就没有x2，整个式子和第二帧图像坐标没关系了呀。 

<p align="center"><img src="https://user-images.githubusercontent.com/58176267/129573787-41318bc8-0052-41cf-8283-0cf716e8cdec.png"></p>  
最后就得到了这个式子：  

<p align="center"><img src="https://user-images.githubusercontent.com/58176267/129573985-33a7f1a5-9e08-4c50-94bc-df16f66b2bdf.png"></p>  
至于作为常数的s1和s2，我们会发现最后不管他们值为多少，都会有上面这个式子，相当于深度信息在对极约束中被“丢掉了”，其实所谓的单目尺度不确定性到底是为什么，通过这么推导下来就很明确了。  
像素坐标p1.p2再带回去就是  

<p align="center"><img src="https://user-images.githubusercontent.com/58176267/129574826-bff2d14a-4304-40f5-ab2d-1496d15012ad.png"></p>  
根据带不带内参K，就有了本质矩阵E和基础矩阵F，也就是下面两个对极约束的形式


 <p align="center"><img src="https://user-images.githubusercontent.com/58176267/129575179-ea97d91e-5536-45b7-bd3b-f3e8051478b3.png"></p>  

#### 本质矩阵E  
这里先补充一些数学知识  
1.3x3的反对称矩阵的秩为2  
设
<p align="center"><img src="https://user-images.githubusercontent.com/58176267/130084577-eacea821-1301-4b4e-b717-4042c0dab629.png"></p>  
则a的反对称矩阵A为  
<p align="center"><img src="https://user-images.githubusercontent.com/58176267/130085311-b1ccad27-0d81-4988-b6a2-1205fd301744.png"></p> 
行初等变换不改变矩阵的秩，先交换行顺序  
<p align="center"><img src="https://user-images.githubusercontent.com/58176267/130085845-60d76be5-e292-4b1b-b546-17b6346cc2e7.png"></p> 
第一行乘a2，第二行乘a3，将第一行加到第二行  
<p align="center"><img src="https://user-images.githubusercontent.com/58176267/130086777-86a6410b-59c7-4ea6-97c3-8acc133b3893.png"></p>  
将第三行乘a1，再加第二行，得到  
<p align="center"><img src="https://user-images.githubusercontent.com/58176267/130087104-e9231849-ad94-4c56-b36c-a1c2818cf109.png"></p>  
秩为2.  
2.关于矩阵的秩  
初等矩阵：由单位矩阵经过一次初等变换得到的矩阵  
初等变换不改变矩阵的秩
可逆矩阵经过初等变换能够变换成单位阵：想一下我们再线代中使用过的求一个矩阵的逆的方法，通过将该矩阵和单位阵组成的一个增广矩阵，经过有限次初等变换后，该矩阵变为单位阵，而右边单位阵的地方现在就是该矩阵的逆矩阵。也就是说，一个矩阵可逆，它一定能通过有限次的初等变换得到。对一个矩阵做一次初等行变换，相当于左侧一个初等矩阵(该矩阵由单位阵做同样的初等行变换得到)。所以，一个可逆矩阵，可以表示成有限个初等矩阵的乘积。例如：  
A为可逆矩阵，左乘B，相当于一系列初等矩阵左乘B，就是对B做一系列初等行变换。A右乘B，相当于一系列初等矩阵右乘B，就是对B做一系列初等列变换
<p align="center"><img src="https://user-images.githubusercontent.com/58176267/130089977-02f334f9-460d-4580-a3f1-9435a09fb8d3.png"></p>  
<p align="center"><img src="https://user-images.githubusercontent.com/58176267/130090723-2202556e-7d59-4ae5-821d-ea5a7f3fc7ef.png"></p>  


本质矩阵E  
因为R为可逆矩，所以
<p align="center"><img src="https://user-images.githubusercontent.com/58176267/130091239-d92a9e66-2bbd-4583-8539-5500c26966eb.png"></p>  

[自由度问题](https://www.zhihu.com/question/270431743)  

#### 基础矩阵F  


3.单应矩阵H的推导  
单应矩阵是假设两帧图像的特征点落在同一个平面上，这样会多一个平面的约束，最后出来的H就是反应了同处于一个平面上的点在两张图像之间的变换关系。它的应用还是比较广泛的我们后面再说。空间中的3D点P的集合共平面，这个平面可以用相机坐标系下的两个参数表示  
<p align="center"><img src="https://user-images.githubusercontent.com/58176267/131254445-7716c8da-cf30-4e2a-9ef2-e6fc312d1734.png"></p>  
这里的n应该为黑体，表示该平面的法向量；d表示相机坐标系原点到该平面的距离  
所以，空间中的点P位于该平面上，则满足：  
<p align="center"><img src="https://user-images.githubusercontent.com/58176267/131254823-17fefd9c-0a9c-427e-9857-480d714adfb1.png"></p>  
即  
<p align="center"><img src="https://user-images.githubusercontent.com/58176267/131254952-9fda4f8c-c77b-4381-ae39-cf6ace3ad444.png"></p>  
由前面的推导  
<p align="center"><img src="https://user-images.githubusercontent.com/58176267/131977123-cd237e25-ed35-47fe-88fa-2f7d46a68012.png"></p>  
整理一下把s2除过来，中间部分记为H  
<p align="center"><img src="https://user-images.githubusercontent.com/58176267/131978176-c6253c0f-48d6-432a-81a3-7e6843234d6a.png"></p>  
注意，由于p1，p2都是齐次坐标，所以常系数有和没有都不影响最终的结果，也就是Hp1和cHp1的效果是一样的，加了常数c最后得到的p2最后相当于都乘了个c，但最后一维要归一化，所以结果一致。这也是为什么H矩阵自由度为8的原因。  





单应矩阵
单应矩阵描述了两个平面之间的映射关系，在两个视图中特征点都处于现实世界的同一平面时，可以用单应矩阵求解两幅图像的变换关系。  
[单应矩阵推导](https://zhuanlan.zhihu.com/p/138266214)  


### 三角化  
由于单目SLAM无法直接获取深度数据，需要通过三角化来恢复空间点，下面是推导过程，代码方面，ORB_SLAM中实现了这部分代码，opencv中cv::triangulatePoints()函数实现过程基本一致，但是最后没有把齐次坐标最后一维归一化。  
<p align="center"><img src="https://user-images.githubusercontent.com/58176267/131513505-904672f0-a828-4efb-805a-44f72730ab0e.png"></p>   
求解该方程：将A进行SVD分解，右奇异值矩阵的最后一列即为问题的解，opencv的SVD分解求出来是Vt，因此取最后一行，注意将最后一维归一化为1才是齐次坐标的形式。





### 1-1卡方检验
理解：如两自由度，当r<5.99时，即有95%的概率(95%的把握)认为服从自由度为2的卡方分布(我们认为只有5%的概率会把内点错误的当成外点?)
卡方检验筛除外点，[假设检验和显著性水平的概念](https://www.matongxue.com/madocs/2095/)，实际上这里的利用卡方检验剔除外点实际上是先认为我们构造的误差和服从卡方分布(对于H来说，是重投影误差，两自由度，对于F来说，是距离，1自由度)。以两自由度为例，如果假设正确，计算的r<=5.99的概率应该为95%，r>5.99的概率应该是很低的，为0.05，那么如果现在有一次实验(根据一对匹配点计算得到了一个r值)，它大于了5.99，那么我们就有很大把握认为我们的假设不正确，但是我们从经验上来说这个假设是对的(相当于是先认为假设是正确的)，那么我们就把这部分点剔除掉，因为这部分匹配点很可能是误匹配的点，才导致r大于了0.05，而不是我们的假设错误。换句话说，如果现在是在进行假设检验，我们由一对匹配点得到了r大于5.99，我们就可以说，原假设很显著的错误了(注意这个很显著的错误，并不是一定错误，是概率上的很大可能)。那反过来，如果我们先说假设是正确的呢，我们认为结果就应该呈卡方分布，那么"实验中"得到了一个r>5.99是不是我们就可以认为这个结果这次实验是显著的错误的呢，结果显著错误不就是我们选的这对匹配点是显著的误匹配吗。
补充，对卡方分布，这个r>5.99的范围叫拒绝域，即拒绝当前卡方分布的假设。
假设检验都有个显著性水平这个概念
可以看下课件卡方分布假设的过程，先有个原假假设和备选假设，求期望，求拒绝域，根据自由度和显著性水平查临界值(阈值)，看检验统计量是否在拒绝域内，做出决策(若在拒绝域内，则拒绝原假设，认为备选假设为真)。  
注意在ORBSLAM2单目初始化中，每次迭代，算出一个H,F，利用卡方检验计算评分时，是双向重投影，其中一向投影时，如果r在拒绝域，则做好是外点的标记并不计入得分，反过来计算另一向时若r不在拒绝域，则计入得分，最终这对匹配点会被标记为外点，但得分算上了一部分(这也容易理解，就是计算出来的H或者F，至少在往其中一幅图像重投影时效果还可以，那么这也算效果好的表现所以得分要加上，但是发现往另一幅图像投影时效果很不好，则最终标记为外点)。  
另外在ORBSLAM2单目初始化中，每次迭代，算出一个H,F，利用卡方检验计算评分时，是双向重投影，其中一向投影时，如果r在拒绝域，则做好是外点的标记并不计入得分，反过来计算另一向时若r不在拒绝域，则计入得分，最终这对匹配点会被标记为外点，但得分算上了一部分(这也容易理解，就是计算出来的H或者F，至少在往其中一幅图像重投影时效果还可以，那么这也算效果好的表现所以得分要加上，但是发现往另一幅图像投影时效果很不好，则最终标记为外点)。



## 视觉词袋Bow  
该部分笔记部分整理自公众号“计算机视觉life”的ORB_SLAM2课程  
Bag of words    
词带：将一幅图变成一个个words    
**流程：**    
* 1.特征点检测，如ORB特征点  
* 2.对特征点描述子进行聚类,得到一棵树（词带树）    
* 3.新来的帧在词带树中遍历，得到很多单词，**记录每个单词的频率**  

**注意**  
叫bag of words 没有叫 list of words 或者 array of words；因为丢弃了word出现的排列的顺序、位置等因素，只考虑出现的频率，简化了表达，节省了存储空间，在分析对比相似度时非常高效；  

**用处：**   

**1.闭环检测**      
判断两幅图片是否是同一场景，即判断图像的相似性；  
类帧差方法（比如直接对应像素相减或用块相减）在视角变化时没办法将两张图像素一一匹配，直接帧差时几乎不能保证是同一像素点的差，同一视角在光照变换时直接用灰度差做也会有问题  

**2.加速匹配**  
这是ORB_SLAM2中使用的一个非常好的技巧，也是词袋容易被忽略的一个用处，就是利用词袋进行加速匹配；  

ORB_SLAM2代码中使用SearchByBoW（用于关键帧跟踪、重定位、闭环检测SIM3计算），以及局部地图里的SearchForTriangulation，内部实现主要利用了BoW中的FeatureVector来加速特征匹配；  

这种方式避免了所有特征点直接进行两两匹配(特征多的时候太费时)，只比较同一结点下的特征点，极大加速了匹配效率，这样做的匹配精度在论文“Bags of Binary Words for Fast Place Recognition in Image Sequences”中结论是效果非常不错；  

**缺点：**  
需要提前加载离线训练好的词袋字典，增加了存储空间，但是带来的优势远大于劣势，有一些改进方法如用二进制存储来压缩词袋，减少存储空间提升加载速度；



## 相关公司
杭州云深处 四足狗 代表产品：绝影
嘉兴：电子科技南湖研究院(AirSim)、






