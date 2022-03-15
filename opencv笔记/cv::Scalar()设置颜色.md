# cv::Scalar()设置颜色  

cv::Scalar的构造函数是cv::Scalar(v1, v2, v3, v4)  
前面的三个参数是依次设置BGR的，和RGB相反，第四个参数设置图片的透明度  

当使用opencv提供的库函数imread()、imwrite()和imshow()时，cv::Scalar(v1, v2, v3, v4)的这四个参数就依次是BGRA，即蓝、绿、红和透明度。  

