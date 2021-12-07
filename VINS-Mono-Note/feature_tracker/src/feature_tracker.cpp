#include "feature_tracker.h"

int FeatureTracker::n_id = 0;

bool inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}

// 根据状态位，进行“瘦身”
// 双指针法 j为慢指针，指向要赋值的位置 i为快指针，只有当status[i] == 1时，才令v[j] = v[i]之后j要后移一位，i是一直遍历的
// 时间复杂度O(N) 空间复杂度O(1)
// 结束以后j位于最后一个元素的后一个位置，正好等于缩减后的元素个数，resize一下就可以了  
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}


FeatureTracker::FeatureTracker()
{
}

// 特征点均匀化
void FeatureTracker::setMask()
{
    if(FISHEYE)
        mask = fisheye_mask.clone();
    else
    //设置掩码 Mat矩阵初始值都是255
        mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));
    

    // prefer to keep features that are tracked for long time
    //外层pair的索引是特征点当前被跟踪的次数track_cnt[i]，对应的值是一个pair，索引是当前帧像素坐标，值是特征点id
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < forw_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(forw_pts[i], ids[i])));
    
    // 利用光流特点，追踪次数多的认为它的稳定性好，排前面
    // sort排序 lamda表达式指定排序规则
    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });
    //为均匀化做准备
    forw_pts.clear();
    ids.clear();
    track_cnt.clear();
    //特征点均匀化，相比于ORB_SLAM四叉树的做法简单直接
    for (auto &it : cnt_pts_id)
    {
        //it.second.first为特征点像素坐标
        if (mask.at<uchar>(it.second.first) == 255)
        {
            // 把挑选剩下的特征点重新放进容器
            //特征点重新放回容器中，包括像素坐标，特征点id，被追踪次数
            forw_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            //一旦某个位置(ix,iy)已经有特征点，取一个以特征点坐标为圆心的圆形区域，区域内mask设为0，后面如果这个区域内原先有特征点的话就不会被重新加入
            // opencv函数，把周围一个圆内全部置0,这个区域不允许别的特征点存在，避免特征点过于集中
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
}

// 把新的点加入容器，id给-1作为区分
void FeatureTracker::addPoints()
{
    for (auto &p : n_pts)
    {
        forw_pts.push_back(p);
        ids.push_back(-1);
        track_cnt.push_back(1);
    }
}

/**
 * @brief 
 * 
 * @param[in] _img 输入图像
 * @param[in] _cur_time 图像的时间戳
 * 1、图像均衡化预处理
 * 2、光流追踪
 * 3、提取新的特征点（如果发布）
 * 4、所有特征点去畸变，计算速度
 */    
void FeatureTracker::readImage(const cv::Mat &_img, double _cur_time)       
{
    cv::Mat img;
    TicToc t_r;
    cur_time = _cur_time;

    if (EQUALIZE)
    {
        // 图像太暗或者太亮，提特征点比较难，所以均衡化一下
        // ! opencv 函数看一下
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        TicToc t_c;
        clahe->apply(_img, img);
        ROS_DEBUG("CLAHE costs: %fms", t_c.toc());
    }
    else
        img = _img;

    // 这里forw表示当前，cur表示上一帧
    if (forw_img.empty())   // 第一次输入图像，prev_img这个没用
    {
        prev_img = cur_img = forw_img = img;
    }
    else
    {
        forw_img = img;
    }

    //当前图像帧特征点信息，新的一帧来了以后作为当前帧，肯定先将这个容器清空  
    forw_pts.clear();
    //cur_img、cur_pts表示的都是上一帧  
    if (cur_pts.size() > 0) // 上一帧有特征点，就可以进行光流追踪了
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;
        // 调用opencv函数进行光流追踪
        // Step 1 通过opencv光流追踪给的状态位剔除outlier
        /**
        * @brief 
        * 光流金字塔
        * @param[in] cur_img 上一帧图像
        * @param[in] forw_img, 当前图像
        * @param[in] cur_pts, 上一帧特征点
        * @param[out] forw_pts,追踪到的当前帧特征点
        * @param[out] status, 标志位 1：特征点被成功追踪   0：特征点追踪失败
        * @param[out] err,未使用
        * @param[in] cv::Size(21, 21),窗口大小
        * @param[in] 3,金字塔层数 3表示共4层金字塔(加上原始层)
        */ 
        
        cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);
        //outliner剔除
        //status中为0的点我们可以认为是outliner,
        //inBorder()函数判断是否在图像边界内
        for (int i = 0; i < int(forw_pts.size()); i++)
            // Step 2 通过图像边界剔除outlier
            if (status[i] && !inBorder(forw_pts[i]))    // 追踪状态好检查在不在图像范围
                status[i] = 0;
        //使用双指针(快慢指针)缩减容器
        reduceVector(prev_pts, status); // 没用到
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);  // 特征点的id
        reduceVector(cur_un_pts, status);   // 去畸变后的坐标
        reduceVector(track_cnt, status);    // 追踪次数
        ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
    }
    // 被追踪到的是上一帧就存在的，因此追踪数+1
    for (auto &n : track_cnt)
        n++;
    //如果要pub这一帧还需要额外的一些操作
    //*1.对级约束剔除外点(调用opencv计算F矩阵的方法，使用了RANSAC剔除外点)
    //*2.特征点均匀化操作,通过mask
    //*3.如果特征点数量不够，提新的点(想后端发布数据时才补充，因为假设向后端发送频率10Hz，图像频率20Hz；每两帧不提新的点是可以的，但是必须要光流追踪)
    if (PUB_THIS_FRAME)
    {
        // Step 3 通过对级约束来剔除outlier
        rejectWithF();
        ROS_DEBUG("set mask begins");
        TicToc t_m;
        //特征点均匀化，相比于ORB_SLAM利用四叉树均匀化的做法简单直接
        setMask();
        ROS_DEBUG("set mask costs %fms", t_m.toc());

        ROS_DEBUG("detect feature begins");
        TicToc t_t;

        //n_max_cnt 需要重新的提取的特征点数组 = MAX_CNT 最大特征点数目 - 去除外点并均匀化后的特征点数目forw_pts.size()
        int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size());
        if (n_max_cnt > 0)
        {
            if(mask.empty()) //没有可以提取的空间了(一般不会发生)
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            if (mask.size() != forw_img.size())
                cout << "wrong size " << endl;
            // 只有发布才可以提取更多特征点，同时避免提的点进mask
            // 会不会这些点集中？会，不过没关系，他们下一次作为老将就得接受均匀化的洗礼
            //输入：当前要提取特征点的图像；MAX_CNT - forw_pts.size()提取个数
            //0.01为接口推荐参数，如果提取到的特征点得分最大为100，那么只要得分大于等于100x0.01 = 1分的就可以被提出来
            //MIN_DIST提取的特征点间欧拉距离的最小值(相当于避免过于集中的特征点)
            //输出：n_pts：提取到的特征点坐标
            cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 0.01, MIN_DIST, mask);
        }
        else
            n_pts.clear();
        ROS_DEBUG("detect feature costs: %fms", t_t.toc());

        ROS_DEBUG("add feature begins");
        TicToc t_a;
        //新提的特征点加入到容器中
        addPoints();
        ROS_DEBUG("selectFeature costs: %fms", t_a.toc());
    }
    //当前帧赋值给上一帧，为下一帧做准备
    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;   // 以上三个量无用
    cur_img = forw_img; // 实际上是上一帧的图像
    cur_pts = forw_pts; // 上一帧的特征点
    //去畸变操作，同时计算特征点的速度(用于后续时间戳标定)
    undistortedPoints();
    prev_time = cur_time;
}

/**
 * @brief 根据对级约束剔除外点  
 * 不太适合每一帧都做这个操作，需要向后端发送数据时为了保证质量，利用这个提出一部分外点  
 */
void FeatureTracker::rejectWithF()
{
    // 当前被追踪到的光流至少8个点
    if (forw_pts.size() >= 8)
    {
        ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_forw_pts(forw_pts.size());
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            // 得到相机归一化坐标系的值
            // 输入一个像素坐标，得到一个去畸变并投影到归一化相机坐标系下的坐标(单目尺度未知，将Z设为1)
            m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            // 这里用一个虚拟相机，原因同样参考https://github.com/HKUST-Aerial-Robotics/VINS-Mono/issues/48
            // 这里有个好-处就是对F_THRESHOLD阈值和相机无关
            // 投影到虚拟相机的像素坐标系
            // 虚拟相机的焦距460，写死了 原因是计算F矩阵时与实际焦距无关，
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;//cx
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;//cy
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        // opencv接口计算本质矩阵，某种意义也是一种对级约束的outlier剔除
        // 这里的目的实际上并不在于得到F矩阵，而是利用RANSAC方法剔除一些外点，可以尝试自己实现  
        cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        int size_a = cur_pts.size();
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(cur_un_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, forw_pts.size(), 1.0 * forw_pts.size() / size_a);
        ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
    }
}

/**
 * @brief 
 * 
 * @param[in] i 
 * @return true 
 * @return false 
 *  给新的特征点赋上id,越界就返回false
 */
bool FeatureTracker::updateID(unsigned int i)
{
    if (i < ids.size())
    {
        if (ids[i] == -1)
            ids[i] = n_id++;
        return true;
    }
    else
        return false;
}

void FeatureTracker::readIntrinsicParameter(const string &calib_file)
{
    ROS_INFO("reading paramerter of camera %s", calib_file.c_str());
    // 读到的相机内参赋给m_camera
    // camodocal::CameraPtr m_camera;  //? camodocal命名空间下定义了Camera虚基类 typedef boost::shared_ptr<Camera> CameraPtr;
    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}

void FeatureTracker::showUndistortion(const string &name)
{
    cv::Mat undistortedImg(ROW + 600, COL + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < COL; i++)
        for (int j = 0; j < ROW; j++)
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
            //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
        }
    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + COL / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + ROW / 2;
        pp.at<float>(2, 0) = 1.0;
        //cout << trackerData[0].K << endl;
        //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < ROW + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < COL + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else
        {
            //ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
        }
    }
    cv::imshow(name, undistortedImg);
    cv::waitKey(0);
}

// 当前帧所有点统一去畸变 liftProjective()函数？，同时计算特征点速度，用来后续时间戳标定
void FeatureTracker::undistortedPoints()
{
    //一定记得先清空容器
    cur_un_pts.clear();
    cur_un_pts_map.clear();
    //cv::undistortPoints(cur_pts, un_pts, K, cv::Mat());
    //这里cur_pts已经被赋值成当前帧了，所以还是对当前帧特征点去几畸变　　
    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
        // 有的之前去过畸变了，这里连同新人重新做一次
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
        Eigen::Vector3d b;
        //得到归一化相机平面坐标
        m_camera->liftProjective(a, b);
        //还是归一化（同除以z）
        cur_un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
        // id->坐标的map
        //key: 当前特征点id    value：归一化x和y坐标点
        cur_un_pts_map.insert(make_pair(ids[i], cv::Point2f(b.x() / b.z(), b.y() / b.z())));
        //printf("cur pts id %d %f %f", ids[i], cur_un_pts[i].x, cur_un_pts[i].y);
    }
    // caculate points velocity
    // 计算特征点速度，需要同一个特征点在上一帧的位置和当前帧的位置
    if (!prev_un_pts_map.empty()) //上一帧这个map不为空就可以计算，为空说明当前可能是第一帧
    {
        double dt = cur_time - prev_time;//时间差Δt
        pts_velocity.clear();
        //遍历当前帧特征点
        for (unsigned int i = 0; i < cur_un_pts.size(); i++)
        {
            //特征点id为-1表示是当前帧刚提取的新的特征点，没法计算速度
            if (ids[i] != -1)
            {
                //遍历当前帧特征点，利用id去找上一帧特征有无id相同的特征点
                std::map<int, cv::Point2f>::iterator it;
                it = prev_un_pts_map.find(ids[i]);
                // 找到同一个特征点 
                if (it != prev_un_pts_map.end())
                {
                    double v_x = (cur_un_pts[i].x - it->second.x) / dt;
                    double v_y = (cur_un_pts[i].y - it->second.y) / dt;
                    // 得到在归一化平面的速度
                    pts_velocity.push_back(cv::Point2f(v_x, v_y));
                }
                else
                    pts_velocity.push_back(cv::Point2f(0, 0));
            }
            else
            {
                pts_velocity.push_back(cv::Point2f(0, 0));
            }
        }
    }
    else
    {
        // 第一帧的情况
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    prev_un_pts_map = cur_un_pts_map;
}
