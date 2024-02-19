# 计算机视觉车道线检测说明文档

[TOC]



## 一.项目介绍

### 1.1、基于python-opencv方法的车道线识别

**基于python-opencv方法的车道线识别**依赖于手工提取的特征来识别，如颜色的特征、结构张量、轮廓等，这些特征还可能与霍夫变换、各种算子或卡尔曼滤波器相结合。在识别车道线之后，采用后处理技术来过滤错误检测并将其分组在一起以形成最终车道。

### 1.2、实现步骤：

1. Canny边缘检测
2. 对特定区域遮罩，提高检测准确度
3. 对车道线轮廓检测并进行上色
4. 计算中心点并输出中心点坐标
5. 计算视频帧数并显示

### 1.3、环境搭建

PyCharm 2021

Python3.8

### 1.4、准备工作

在默认环境中安装后续代码依赖的第三方包

OpenCV: `pip install opencv-contrib-python -i https://pypi.tuna.tsinghua.edu.cn/simple/`

Matplotlib: `pip install matplotlib`

Sklearn: `pip install sklearn`

Labelme: `pip install labelme`

Imutils: `pip install imutils`

### 1.5、代码结构和各部分功能如下：

`roadcv.py`

```bash
├── do_canny# 边缘检测
├── do_segment# 区域遮罩
├── do_color#对车道线进行轮廓识别
├── do_center#处理中心点
├── image_resize #修改图片大小
```

## 二.性能分析

fps 平均帧数达到了50帧/秒与原视频帧数几乎相等

## 三.创新说明

1. 由于霍夫直线检测方法准确但不能做弯道检测，拟合方法可以检测弯道但不稳定，透视变换操作会对相机有一些具体的要求，在变换前需要调正图像，而且摄像机的安装和道路本身的倾斜都会影响变换效果。所以我们对边缘检测和区域遮罩后只含有车道线的图像进行轮廓识别处理上色，计算中心点坐标，排除外在影响
2. 对于不同性能的使用要求，我们可以通过调节视频的播放倍速来改变视频帧数的多少，从而达到多种检测需求
3. 对区域遮罩后的图像再进行高斯滤波减噪和阈值处理，检测车道线轮廓，计算检测到的车道线轮廓的中心点坐标，在一个新的图像中绘制出检测到的中心点，并在终端中输出每帧的中心点坐标数组         
4. 通过resize函数来更改输出的图像大小，以便于更好的对比观察识别出的车道线图像

## 四.未来优化

1. 将提取出的中心点轮廓连接后与原车道线融合
2. 将上色后的图像叠加到原图像中

## 五.总结

这次的计算机视觉-车道线检测我们采用的是基于传统方法的车道线检测，使用opencv-python代码来实现车道线检测，完成对轮廓的识别处理，计算中心点坐标并输出坐标数组，计算视频平均帧数并显示。

优点：基于传统方法的车道线检测方法简单，速度较快。不需要构建神经网络，硬件成本低。

缺点：

1. 易受光线、环境、道路车辆等因素的影响，使得结果有偏差；
2. 需要调整的参数较多

效果视频地址如下【【智能车航天物流组】opencv车道线中点识别并提取坐标点-哔哩哔哩】 https://b23.tv/Hthvchw

## 六.参考文献

1、[Python] OpenCV+霍夫直线检测的车道线识别

https://www.52pojie.cn/thread-1473150-1-1.html

2、cv2.imshow调整窗口大小

https://blog.csdn.net/weixin_42929622/article/details/115273454

3、传统车道线检测-canny边缘检测-霍夫变换-完整代码（python）

https://blog.csdn.net/m0_46988935/article/details/109234900

4、OpenCV 图像缩放：cv.resize() 函数详解

https://blog.csdn.net/hysterisis/article/details/112381220

5、cv2.findContours() 轮廓检测

https://blog.csdn.net/Easen_Yu/article/details/89365497

6、python用opencv实现直线检测和计算交点

https://blog.csdn.net/qq_33004317/article/details/100079230