/**
 * @file Viewer.cc
 * @author wangrui (wangrui957@163.com)
 * @brief 可视化查看器的实现
 * @version 1.0
 * @date 2021-10-27
 * 
 * @copyright Copyright (c) 2021 wangrui
 * 
 */



#include "Viewer.h"
#include <pangolin/pangolin.h>

Viewer::Viewer()
{

}

//多线程的主函数
void Viewer::Run()
{
    
    pangolin::CreateWindowAndBind("SLAM_Viewer",1024, 768);

    // 开启深度测试 OpenGL绘制时，会检查要绘制像素前是否有“遮挡”，有的话就不绘制，避免出现透视情况
    glEnable(GL_DEPTH_TEST);

    // 使用颜色混合
    glEnable(GL_BLEND);

    // 颜色混合选项设置，该方式表示源颜色乘自身alpha值，目标颜色颜色乘1.0减源颜色的alpha值；
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // 创建观察相机视图
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, mViewerF, mViewerF, 512, 384, 0.1, 1000),
        pangolin::ModelViewLookAt(mViewerPointX, mViewerPointY, mViewerPointZ, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    // 定义——显示面板
    //范围是“相机视图”的全部范围，显示长宽比-1024.0f/768.0f
    pangolin::View& d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f/768.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));

    while(1)
    {
        //清除缓冲区颜色和深度缓存
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        //
        d_cam.Activate(s_cam);

        //先画一个矩形显示 
        
        //线宽
        glLineWidth(3);

        //颜色：绿色  
        glColor3f(0.0f, 1.0f, 0.0f);

        //使用线将下面各顶点相连
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(1, 0, 0);
        glEnd();

        //运行帧循环，推进窗口事件  
        pangolin::FinishFrame();
    }

}




