# cv::cvtColor  将图像从一个颜色空间转换为另一个颜色空间

#include <opencv2/imgproc.hpp>

```cpp
void cv::cvtColor	(	InputArray 	src,
OutputArray 	dst,
int 	code,
int 	dstCn = 0 
)		

Python:
cv.cvtColor(	src, code[, dst[, dstCn]]	) ->	dst
``` 

## 参数

src 输入图像：8位无符号，16位无符号（CV_16UC ...）或单精度浮点。

dst输出图像 ：src相同的大小和深度。

代码颜色空间转换代码（参见ColorConversionCodes）。

dstCn目的图像中的通道数 ： 如果参数为0，则通道数数量是从src中自动导出的。

## 注意  
* 在从RGB颜色空间转换的情况下，应明确指定通道的顺序（RGB或BGR）  
* OpenCV中的默认颜色格式通常称为RGB，但实际上是BGR。因此，标准（24位）彩色图像中的第一个字节将是8位蓝色，第二个字节将是绿色的，第三个字节将为红色

R，G和B通道值的传统范围是  
* 0 to 255 for CV_8U images
* 0 to 65535 for CV_16U images
* 0 to 1 for CV_32F images
