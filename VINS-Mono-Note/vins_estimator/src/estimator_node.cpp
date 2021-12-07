#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "estimator.h"
#include "parameters.h"
#include "utility/visualization.h"


Estimator estimator;

std::condition_variable con;
double current_time = -1;
queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
queue<sensor_msgs::PointCloudConstPtr> relo_buf;
int sum_of_wait = 0;

std::mutex m_buf;
std::mutex m_state;
std::mutex i_buf;
std::mutex m_estimator;

double latest_time;
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;
Eigen::Vector3d gyr_0;
bool init_feature = 0;
bool init_imu = 1;
double last_imu_t = 0;

/**
 * @brief 根据当前imu数据预测当前位姿
 * 注意这里预测的起点都是以后端优化的结果为起点的，所以结果比较准确，同时频率给提高到了和IMU频率相同
 * 
 * @param[in] imu_msg 
 */
void predict(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    if (init_imu) //第一帧imu的标志位，初始为0，进入后只把latest_time赋值为当前imu的时间，之后将标志位置0
    {
        latest_time = t;
        init_imu = 0;
        return;
    }

    double dt = t - latest_time;
    latest_time = t;

    // 得到加速度
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};
    // 得到角速度
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};
    // 上一时刻世界坐标系下加速度值
    //tmp_Ba加速度计零偏
    //tmp_Q:上一时刻(帧)imu在世界系下的姿态   acc_0:上一帧imu加速度值(算完之后在后面会重新赋值)
    //tmp_Q * (acc_0 - tmp_Ba)得到了上一帧世界系下的加速度值(IMU系转到world)，之后减去重力，得到了世界系下没有重力影响
    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - estimator.g;  //Estimator estimator;  Estimator类下public成员Vector3d g;

    // 中值陀螺仪的结果(上一帧和这一帧陀螺仪值)
    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
    
    // 更新姿态 (先更新当前姿态因为后面更新当前加速度需要用到)
    // un_gyr * dt陀螺仪中值*dt 得到角度变化量
    // Utility::deltaQ类下的deltaQ()函数
    // 姿态变换量(四元数表示)右乘上一帧姿态，得到当前帧姿态
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);
    
    // 当前时刻世界坐标系下的加速度值
    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - estimator.g;
    // 加速度中值积分的值(世界坐标系下)
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    
    // 经典物理中位置，速度更新方程
    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;
    
    //当前加速度和陀螺仪值赋值成上一时刻的值，以便下次使用
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}
// 用最新VIO结果更新最新imu对应的位姿
void update()
{
    TicToc t_predict;
    latest_time = current_time;
    tmp_P = estimator.Ps[WINDOW_SIZE];
    tmp_Q = estimator.Rs[WINDOW_SIZE];
    tmp_V = estimator.Vs[WINDOW_SIZE];
    tmp_Ba = estimator.Bas[WINDOW_SIZE];
    tmp_Bg = estimator.Bgs[WINDOW_SIZE];
    acc_0 = estimator.acc_0;
    gyr_0 = estimator.gyr_0;

    queue<sensor_msgs::ImuConstPtr> tmp_imu_buf = imu_buf;  // 遗留的imu的buffer，因为下面需要pop，所以copy了一份
    for (sensor_msgs::ImuConstPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
        // 得到最新imu时刻的可靠的位姿
        predict(tmp_imu_buf.front());

}

// 获得匹配好的图像imu组
std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>>
getMeasurements()
{
    std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;

    while (true) //注意这里是个死循环，取符合要求的imu数据和图像数据组成pair，并放到measurements数组，知道两个buffer有空的或者不满足时间戳要求了就把measurements返回
    {
        if (imu_buf.empty() || feature_buf.empty())//有一个是空的无法对齐  
            return measurements;
        // imu   *******
        // image          *****
        // 这就是imu还没来
        // 希望 imu最后一个时间戳大于图像第一帧时间戳(补偿时间差后的)，此时才有可能满足对齐操作的条件(两帧间有imu数据)
        // 因此如果满足取反，imu最后一个时间戳小于等于图像第一帧时间戳(补偿时间差后的)，没办法对齐，需要等待imu进来，使图像帧之间含有imu数据
        if (!(imu_buf.back()->header.stamp.toSec() > feature_buf.front()->header.stamp.toSec() + estimator.td))
        {
            //ROS_WARN("wait for imu, only should happen at the beginning");
            sum_of_wait++;
            return measurements;
        }
        // imu        ****
        // image    ******
        // 这种只能扔掉一些image帧
        // 希望 imu第一个时间戳小于图像第一帧时间戳(补偿时间差后的)，因为第一个图像帧没法找到它之前的对应的imu数据？
        // vins取关联数据是取这一帧图像之前的imu数据
        if (!(imu_buf.front()->header.stamp.toSec() < feature_buf.front()->header.stamp.toSec() + estimator.td))
        {
            ROS_WARN("throw img, only should happen at the beginning");
            feature_buf.pop();
            continue;
        }
        // 此时就保证了图像前一定有imu数据
        sensor_msgs::PointCloudConstPtr img_msg = feature_buf.front();
        feature_buf.pop();
        // 一般第一帧不会严格对齐，但是后面就都会对齐，当然第一帧也不会用到(在第一帧图像来之前可能累积了过多的imu数据，但是这部分数据和第一帧图像在滑窗中没有使用)
        // 取出第一帧图像前的imu数据放到IMUs
        std::vector<sensor_msgs::ImuConstPtr> IMUs;
        while (imu_buf.front()->header.stamp.toSec() < img_msg->header.stamp.toSec() + estimator.td)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        // 保留图像时间戳后一个imu数据，但不会从buffer中扔掉，因为后面图像帧还是需要的
        // imu    *   *
        // image    *
        // 刚刚是把第一帧图像时间戳之前的imu数据从imu_buf拿出来放到了IMUs，现在imu_buf中第一个imu数据的时间戳在刚在拿出的第一帧图像img_msg时间戳的后面，也放到IMUs中
        // 目的是把IMUs中最后两个imu数据插值得到这帧图像对应的imu数据(更合适)，如上图
        IMUs.emplace_back(imu_buf.front());
        if (IMUs.empty())
            ROS_WARN("no imu between two image");
        measurements.emplace_back(IMUs, img_msg);
    }
    return measurements;
}


/**
 * @brief imu消息存进buffer，同时按照imu频率预测位姿并发送，这样就可以提高里程计频率
 * 
 * @param[in] imu_msg 
 */
void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    //时间戳判断
    if (imu_msg->header.stamp.toSec() <= last_imu_t)
    {
        ROS_WARN("imu message in disorder!");
        return;
    }
    last_imu_t = imu_msg->header.stamp.toSec();
    
    // 线程锁 条件变量用法 （https://www.jianshu.com/p/c1dfa1d40f53）
    //imu回调函数，需要将imu数据加入一个队列，另外有线程需要将imu数据从队列中取出imu数据；需要通过线程锁，保证放的时候不能取，取的时候不能放
    //条件变量，保证cpu占用率不会过高；加完数据“通知”需要取数据的线程，取数据的线程如果没有收到“通知”就进入休眠状态，不占用cpu资源
    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
    con.notify_one();

    last_imu_t = imu_msg->header.stamp.toSec();

    {
        std::lock_guard<std::mutex> lg(m_state);
        //预测：根据当前imu数据预测当前位姿(P,V,Q)
        predict(imu_msg);
        std_msgs::Header header = imu_msg->header;
        header.frame_id = "world";
        // 只有初始化完成后才发送当前结果
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);
    }
}

/**
 * @brief 将前端信息送进buffer
 * 
 * @param[in] feature_msg 
 */
void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    if (!init_feature)
    {
        //skip the first detected feature, which doesn't contain optical flow speed
        init_feature = 1;
        return;
    }
    m_buf.lock();
    feature_buf.push(feature_msg);
    m_buf.unlock();
    con.notify_one();
}

/**
 * @brief 将vins估计器复位
 * 
 * @param[in] restart_msg 
 */

void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        m_buf.lock();
        while(!feature_buf.empty())
            feature_buf.pop();
        while(!imu_buf.empty())
            imu_buf.pop();
        m_buf.unlock();
        m_estimator.lock();
        estimator.clearState();
        estimator.setParameter();
        m_estimator.unlock();
        current_time = -1;
        last_imu_t = 0;
    }
    return;
}

void relocalization_callback(const sensor_msgs::PointCloudConstPtr &points_msg)
{
    //printf("relocalization callback! \n");
    m_buf.lock();
    relo_buf.push(points_msg);
    m_buf.unlock();
}

// thread: visual-inertial odometry
void process()
{
    while (true)    // 这个线程是会一直循环下去
    {
        std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;
        std::unique_lock<std::mutex> lk(m_buf);
        //这里相当于一直给imu和图像buffer上锁等待(不再放数据了？)，getMeasurements()函数去除了buffer中满足要求的数据后(measurements.size()!=0)再解锁，执行下面的流程
        con.wait(lk, [&]
                 {
            return (measurements = getMeasurements()).size() != 0;
                 });
        lk.unlock();    // 数据buffer的锁解锁，回调可以继续塞数据了
        m_estimator.lock(); // 进行后端求解，不能和复位重启冲突
        // 给予范围的for循环，这里就是遍历每组image imu组合
        for (auto &measurement : measurements)
        {
            //这里面每一个measurement是个pair
            auto img_msg = measurement.second;
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
            // 遍历imu
            for (auto &imu_msg : measurement.first)
            {
                //与上面img_msg对应的每一个imu数据
                double t = imu_msg->header.stamp.toSec();
                double img_t = img_msg->header.stamp.toSec() + estimator.td;
                if (t <= img_t) //imu时间戳在图像时间戳之前，不需要做额外处理
                { 
                    if (current_time < 0)
                        current_time = t;
                    double dt = t - current_time; //当前imu时间t与上一个imu时间差
                    ROS_ASSERT(dt >= 0);
                    current_time = t; //
                    dx = imu_msg->linear_acceleration.x;
                    dy = imu_msg->linear_acceleration.y;
                    dz = imu_msg->linear_acceleration.z;
                    rx = imu_msg->angular_velocity.x;
                    ry = imu_msg->angular_velocity.y;
                    rz = imu_msg->angular_velocity.z;
                    // 时间差和imu数据送进去，时间差是当前取出的这帧imu与上帧imu的时间差
                    // 后端接口，处理imu数据
                    estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);

                }
                else    // 这就是针对最后一个imu数据，需要做一个简单的线性插值
                {
                    double dt_1 = img_t - current_time;
                    double dt_2 = t - img_t;
                    current_time = img_t;
                    ROS_ASSERT(dt_1 >= 0);
                    ROS_ASSERT(dt_2 >= 0);
                    ROS_ASSERT(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
                    dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
                    dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
                    rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                    ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                    rz = w1 * rz + w2 * imu_msg->angular_velocity.z;
                    estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
                }
            }
            // set relocalization frame
            // 回环相关部分
            sensor_msgs::PointCloudConstPtr relo_msg = NULL;
            while (!relo_buf.empty())   // 取出最新的回环帧
            {
                relo_msg = relo_buf.front();
                relo_buf.pop();
            }
            if (relo_msg != NULL)   // 有效回环信息
            {
                vector<Vector3d> match_points;
                double frame_stamp = relo_msg->header.stamp.toSec();    // 回环的当前帧时间戳
                for (unsigned int i = 0; i < relo_msg->points.size(); i++)
                {
                    Vector3d u_v_id;
                    u_v_id.x() = relo_msg->points[i].x; // 回环帧的归一化坐标和地图点idx
                    u_v_id.y() = relo_msg->points[i].y;
                    u_v_id.z() = relo_msg->points[i].z;
                    match_points.push_back(u_v_id);
                }
                // 回环帧的位姿
                Vector3d relo_t(relo_msg->channels[0].values[0], relo_msg->channels[0].values[1], relo_msg->channels[0].values[2]);
                Quaterniond relo_q(relo_msg->channels[0].values[3], relo_msg->channels[0].values[4], relo_msg->channels[0].values[5], relo_msg->channels[0].values[6]);
                Matrix3d relo_r = relo_q.toRotationMatrix();
                int frame_index;
                frame_index = relo_msg->channels[0].values[7];
                estimator.setReloFrame(frame_stamp, frame_index, match_points, relo_t, relo_r);
            }

            ROS_DEBUG("processing vision data with stamp %f \n", img_msg->header.stamp.toSec());

            TicToc t_s;
            // 特征点id->特征点信息
            map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image;
            for (unsigned int i = 0; i < img_msg->points.size(); i++)
            {
                int v = img_msg->channels[0].values[i] + 0.5;
                int feature_id = v / NUM_OF_CAM;
                int camera_id = v % NUM_OF_CAM;
                double x = img_msg->points[i].x;    // 去畸变后归一滑像素坐标
                double y = img_msg->points[i].y;
                double z = img_msg->points[i].z;
                double p_u = img_msg->channels[1].values[i];    // 特征点像素坐标
                double p_v = img_msg->channels[2].values[i];
                double velocity_x = img_msg->channels[3].values[i]; // 特征点速度
                double velocity_y = img_msg->channels[4].values[i];
                ROS_ASSERT(z == 1); // 检查是不是归一化
                Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                image[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
            }
            estimator.processImage(image, img_msg->header);

            // 一些打印以及topic的发送
            double whole_t = t_s.toc();
            printStatistics(estimator, whole_t);
            std_msgs::Header header = img_msg->header;
            header.frame_id = "world";

            pubOdometry(estimator, header);
            pubKeyPoses(estimator, header);
            pubCameraPose(estimator, header);
            pubPointCloud(estimator, header);
            pubTF(estimator, header);
            pubKeyframe(estimator);
            if (relo_msg != NULL)
                pubRelocalization(estimator);
            //ROS_ERROR("end: %f, at %f", img_msg->header.stamp.toSec(), ros::Time::now().toSec());
        }
        m_estimator.unlock();
        m_buf.lock();
        m_state.lock();
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            update();
        m_state.unlock();
        m_buf.unlock();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    readParameters(n);
    estimator.setParameter();
#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif
    ROS_WARN("waiting for image and imu...");

    // 注册一些publisher
    registerPub(n);
    // 接受imu消息
    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    // 接受前端视觉光流结果
    ros::Subscriber sub_image = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
    // 接受前端重启命令
    ros::Subscriber sub_restart = n.subscribe("/feature_tracker/restart", 2000, restart_callback);
    // 回环检测的fast relocalization响应
    ros::Subscriber sub_relo_points = n.subscribe("/pose_graph/match_points", 2000, relocalization_callback);

    // 核心处理线程
    std::thread measurement_process{process};
    ros::spin();

    return 0;
}
