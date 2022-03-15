# 在图像中画圆——cv::circle()  




## 功能  

**cv::circle绘制具有给定中心和半径的简单或填充的圆圈（空心或实心）**  

#include <opencv2/imgproc.hpp>

```cpp
void cv::circle	(	InputOutputArray 	img,
Point 	center,
int 	radius,
const Scalar &  color,
int 	thickness = 1,
int 	lineType = LINE_8,
int 	shift = 0 
)

Python:
cv.circle(	img, center, radius, color[, thickness[, lineType[, shift]]]	) ->	img
```  

## 参数

img    绘制圆的图像

center 圆的中心

radius 圆的半径

color  彩色圆形颜色

thickness  圆圈轮廓的厚度    圆形轮廓的粗细 (如果为正)。负值 (如 cv::FILLED) 表示要绘制实心圆

lineType  圆形边界的线型类型  可以具体看下有什么区别，一般情况下不指定用默认即可  

shift  中心坐标和半径值中的小数位数  



