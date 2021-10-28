/**
 * @file Viewer.cc
 * @author wangrui (wangrui957@163.com)
 * @brief pangolin测试主函数
 * @version 1.0
 * @date 2021-10-27
 * 
 * @copyright Copyright (c) 2021 wangrui
 * 
 */


#include <thread>
#include "Viewer.h"

int main(int argc, char* argv[])
{

    Viewer* pViewer;

    std::thread *ptViewer = new std::thread(&Viewer::Run, pViewer);

    ptViewer->join();

    cout << "子线程执行完毕，继续执行主线程： " << endl;
	
    return 0;
    
}