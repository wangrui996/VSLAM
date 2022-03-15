# cv::arrowedLine()绘制箭头  

#include <opencv2/imgproc.hpp>

绘制从第一个点到第二个指向的箭头段。  
功能cv::arrowedLine()在图像中的PT1和PT2点之间绘制箭头。  

```cpp
void cv::arrowedLine	(	InputOutputArray 	img,
Point 	pt1,
Point 	pt2,
const Scalar & 	color,
int 	thickness = 1,
int 	line_type = 8,
int 	shift = 0,
double 	tipLength = 0.1 
)		
Python:
cv.arrowedLine(	img, pt1, pt2, color[, thickness[, line_type[, shift[, tipLength]]]]	) ->	img
```  

## 参数

img 图像  

pt1 箭头开始的点  

pt2 箭头指向的点  

color 彩色线颜色

thickness 线厚度

line_type 线型类型  

shift 点坐标中的小数位数  

tiplength  箭头尖端的长度  与箭头长度相关

