/**
 * @file Viewer.h
 * @author wangrui (wangrui957@163.com)
 * @brief 可视化查看器的声明
 * @version 1.0
 * @date 2021-10-26
 * 
 * @copyright Copyright (c) 2021 wangrui
 * 
 */




#ifndef VIEWER_H
#define VIEWER_H

  

/**
 *@brief 可视化查看器的声明
 */
class Viewer
{
public:
    /**
     * 构造函数
     */
    Viewer();
    
    /**
     * @brief 进程主函数、绘制相机、轨迹与地图点等
     */
    void Run();

private:

    //可视化窗口的视角  观察相机的位置、焦距
    float mViewerPointX, mViewerPointY, mViewerPointZ, mViewerF;
    

};

#endif // VIEWER_H