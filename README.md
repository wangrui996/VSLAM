# VSLAM
SLAM学习过程记录

* [1-SLAM基础知识](#1-SLAM基础知识)


## 1-SLAM基础知识

### 1.1 卡方检验
理解：如两自由度，当r<5.99时，即有95%的概率(95%的把握)认为服从自由度为2的卡方分布(我们认为只有5%的概率会把内点错误的当成外点?)
卡方检验筛除外点，[假设检验和显著性水平的概念](https://www.matongxue.com/madocs/2095/)，实际上这里的利用卡方检验剔除外点实际上是先认为我们构造的误差和服从卡方分布(对于H来说，是重投影误差，两自由度，对于F来说，是距离，1自由度)。以两自由度为例，如果假设正确，计算的r<=5.99的概率应该为95%，r>5.99的概率应该是很低的，为0.05，那么如果现在有一次实验(根据一对匹配点计算得到了一个r值)，它大于了5.99，那么我们就有很大把握认为我们的假设不正确，但是我们从经验上来说这个假设是对的(相当于是先认为假设是正确的)，那么我们就把这部分点剔除掉，因为这部分匹配点很可能是误匹配的点，才导致r大于了0.05，而不是我们的假设错误。换句话说，如果现在是在进行假设检验，我们由一对匹配点得到了r大于5.99，我们就可以说，原假设很显著的错误了(注意这个很显著的错误，并不是一定错误，是概率上的很大可能)。那反过来，如果我们先说假设是正确的呢，我们认为结果就应该呈卡方分布，那么"实验中"得到了一个r>5.99是不是我们就可以认为这个结果这次实验是显著的错误的呢，结果显著错误不就是我们选的这对匹配点是显著的误匹配吗。
补充，对卡方分布，这个r>5.99的范围叫拒绝域，即拒绝当前卡方分布的假设。
假设检验都有个显著性水平这个概念
可以看下课件卡方分布假设的过程，先有个原假假设和备选假设，求期望，求拒绝域，根据自由度和显著性水平查临界值(阈值)，看检验统计量是否在拒绝域内，做出决策(若在拒绝域内，则拒绝原假设，认为备选假设为真)。  
注意在ORBSLAM2单目初始化中，每次迭代，算出一个H,F，利用卡方检验计算评分时，是双向重投影，其中一向投影时，如果r在拒绝域，则做好是外点的标记并不计入得分，反过来计算另一向时若r不在拒绝域，则计入得分，最终这对匹配点会被标记为外点，但得分算上了一部分(这也容易理解，就是计算出来的H或者F，至少在往其中一幅图像重投影时效果还可以，那么这也算效果好的表现所以得分要加上，但是发现往另一幅图像投影时效果很不好，则最终标记为外点)。  
另外在ORBSLAM2单目初始化中，每次迭代，算出一个H,F，利用卡方检验计算评分时，是双向重投影，其中一向投影时，如果r在拒绝域，则做好是外点的标记并不计入得分，反过来计算另一向时若r不在拒绝域，则计入得分，最终这对匹配点会被标记为外点，但得分算上了一部分(这也容易理解，就是计算出来的H或者F，至少在往其中一幅图像重投影时效果还可以，那么这也算效果好的表现所以得分要加上，但是发现往另一幅图像投影时效果很不好，则最终标记为外点)。















# ORB-SLAM2
**Authors:** [Raul Mur-Artal](http://webdiis.unizar.es/~raulmur/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/), [J. M. M. Montiel](http://webdiis.unizar.es/~josemari/) and [Dorian Galvez-Lopez](http://doriangalvez.com/) ([DBoW2](https://github.com/dorian3d/DBoW2))

**13 Jan 2017**: OpenCV 3 and Eigen 3.3 are now supported.

**22 Dec 2016**: Added AR demo (see section 7).

ORB-SLAM2 is a real-time SLAM library for **Monocular**, **Stereo** and **RGB-D** cameras that computes the camera trajectory and a sparse 3D reconstruction (in the stereo and RGB-D case with true scale). It is able to detect loops and relocalize the camera in real time. We provide examples to run the SLAM system in the [KITTI dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) as stereo or monocular, in the [TUM dataset](http://vision.in.tum.de/data/datasets/rgbd-dataset) as RGB-D or monocular, and in the [EuRoC dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) as stereo or monocular. We also provide a ROS node to process live monocular, stereo or RGB-D streams. **The library can be compiled without ROS**. ORB-SLAM2 provides a GUI to change between a *SLAM Mode* and *Localization Mode*, see section 9 of this document.

<a href="https://www.youtube.com/embed/ufvPS5wJAx0" target="_blank"><img src="http://img.youtube.com/vi/ufvPS5wJAx0/0.jpg" 
alt="ORB-SLAM2" width="240" height="180" border="10" /></a>
<a href="https://www.youtube.com/embed/T-9PYCKhDLM" target="_blank"><img src="http://img.youtube.com/vi/T-9PYCKhDLM/0.jpg" 
alt="ORB-SLAM2" width="240" height="180" border="10" /></a>
<a href="https://www.youtube.com/embed/kPwy8yA4CKM" target="_blank"><img src="http://img.youtube.com/vi/kPwy8yA4CKM/0.jpg" 
alt="ORB-SLAM2" width="240" height="180" border="10" /></a>
