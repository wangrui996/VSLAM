# 对cv::Mat增加行或列.md  

##  hconcat()  水平的拼接两个矩阵
https://docs.opencv.org/3.4.1/d2/de8/group__core__array.html#gaf9771c991763233866bf76b5b5d1776f）

对给定矩阵应用水平连接。
该函数水平地连接两个或更多个CV ::垫矩阵（以相同的行数）

### 1.水平连接两个Mat  

可以用来水平拼接两个图像  

```cpp
void cv::hconcat	(	InputArray 	src1,
InputArray 	src2,
OutputArray 	dst 
)	
```

```cpp
cv::Mat_<float> A = (cv::Mat_<float>(3, 2) << 1, 4,
                                              2, 5,
                                              3, 6);
cv::Mat_<float> B = (cv::Mat_<float>(3, 2) << 7, 10,
                                              8, 11,
                                              9, 12);
cv::Mat C;
cv::hconcat(A, B, C);
//C:
//[1, 4, 7, 10;
// 2, 5, 8, 11;
// 3, 6, 9, 12]
```

### 2.

```cpp
void cv::hconcat	(	const Mat * 	src,
size_t 	nsrc,
OutputArray 	dst 
)	
```
```cpp
cv::Mat matArray[] = { cv::Mat(4, 1, CV_8UC1, cv::Scalar(1)),
 cv::Mat(4, 1, CV_8UC1, cv::Scalar(2)),
 cv::Mat(4, 1, CV_8UC1, cv::Scalar(3)),};
cv::Mat out;
cv::hconcat( matArray, 3, out );
//out:
//[1, 2, 3;
// 1, 2, 3;
// 1, 2, 3;
// 1, 2, 3]
```


##  vconcat() 垂直拼接两个矩阵  



