# 中央任务调度任务—通信开发

 [TOC]



## 1.项目算法功能

我们的这个算法能够实现Windows和Ubuntu下信息传递，可以传递中文，英文，数字等信息，为后续的进程提供了保障。                    

`ping.c`(Ubuntu端源代码)

在Ubuntu端的代码中，我们设置了当接受到的字符串长度小于0时，显示Server Receive Data Failed！，当接收到的字符串以‘q’开头时，则显示Quit！后自动退出，当接收到正常的字符串时，则显示接收到的内容。

`源2.cpp`（Windows端源代码）

在Windows端的代码中，我们设置了当成功连接socket后显示Socket Connect Succeed! Input Data:

## 2.项目实践过程

第一步：

更改网络IP地址，将Windows端IP地址改成192.168.0.2；子网掩码自动生成为255.255.255.0。

然后将Ubuntu端IP地址改成192.168.0.1；子网掩码改为255.255.255.0。

第二步：

编写`ping.c`(Ubuntu端源代码)和`源2.cpp`（Windows端源代码）

第三步：

编译我们写好的代码，windows端我们用的VS2015编译

Ubuntu端我们用的gcc编译:`gcc -o ping ping.c`

第四步：

我们进行了通信前的测试，首先从windows端向Ubuntu端ping

 `ping 192.168.0.1`

然后从Ubuntu端向windows端ping

 `ping 192.169.0.1`

第五步：

先打开Ubuntu终端，运行我们编译的可执行程序，

 `./ping`

然后再打开VS2015，运行客户端程序

现在我们成功建立起了Windows和Ubuntu的通信。

第六步：

Windows端输入我们需要传递的字符，

可以看到我们的Ubuntu端成功接收到字符。

我们的Windows与Ubuntu的通信到这就已经完全成功了。



## 3.创新点

1.我们在实践的过程中发现由Ubuntu端向Windows端ping时输入192.168.0.2并没有成功，但经过我们的摸索发现应该输入192.168.0.1才能ping成功。

2.在windows端输入中文时Ubuntu端接收到的是乱符，我们了解到的就是因为Windows与Ubuntu编码格式不一样，就只需要在终端的上面工具栏里点击终端，选择第一个设定字符编码，然后选择“简体中文--GBK”即可。

3.我们也尝试过两台Ubuntu设备之间的ssh通信，在实践过程中，我们能明显的感觉到两台Ubuntu设备之间的ssh通信要比Windows与Ubuntu的socket通信更加便捷，可以直接打开服务端的终端进行操作，而不是简单的传输数据，并且ssh有着较好的安全性。

 

## 4.总结

 这次的Windows与Ubuntu的通信我们选择的是VS2015作为上位机的开发工具，在这之前我们也没有接触过这款开发工具，完全是看到任务后开始学习的，所以我们在完成这个任务的过程中遇到了很多问题，有一些问题是可以在网上找到解决办法的，但有的问题却找不到解决办法，甚至是我们因为自己设备的问题而遇到了别人不会遇到的错误，这个时候我们只有自己去尝试，去摸索，在一遍遍的操作后才解决这些问题，成功完成了给定的任务。这让我们学会了如何去学习一些自己以前没接触过的东西，也加强了我们的自主探索能力。

讲解视频地址如下：【window_ros 进行通信并消息互传-哔哩哔哩】 https://b23.tv/MHCl8XS

 

## 5. 参考文献

【1】使用socket 情况下的windows系统与ubuntu16.04系统通信详解.CSDN.Robot_hfut

【2】 Ubuntu下进行ssh 详解.CSDN.五新

 

# 计算机视觉—车道线检测

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



Astar_searcher.cpp中仅修改了启发函数
并将不同的启发函数运行并路径规划得到不同的结果截图为Astar实验截图

# **在ROS中实现A\*路径规划**

[TOC]

## **1.**  **项目介绍**

（1）A*算法，A*（A _Star)算法是一种静态路网中求解最短路径最有效的**直接**搜索方法，也是解决许多搜索问题的有效算法。算法中的距离估算值与实际值越接近，最终搜素速度越快。该算法在最短路径搜索法中，分类为直接搜索算法，启发式算法，静态图搜索算法。

（2）算法分类

直接搜索算法；是在地图上进行搜索，不经过任何预处理；

启发式算法：通过启发函数引导算法的搜索方向；（在此使用启发式算法）

静态图搜索算法：被搜索的图的权值不随时间变化（后被证明同样可以适用于动态图的搜索）

（3）A*算法优点

能够求解出状态空间搜素的最短路径，也就是用最快的方法求解问题，A*就是干这种事情的。

（4）A*算法的模型

f(n)=g(n)+h(n)

其中，f(n)是总的搜索代价，g(n)是从起点到当前节点n的代价和，h(n)是从当前节点n到目标节点的最优代价启发函数。



## 2、估值函数及优化

（1）曼哈顿距离：标准的启发式函数（Manhattan distance）

算法实现

```cpp
Heuristics 1: Manhattan

h = std::abs(node1_coord(0) - node2_coord(0) ) +

	std::abs(node1_coord(1) - node2_coord(1) ) +

	std::abs(node1_coord(2) - node2_coord(2) );
```

（2）欧几里得：

算法实现

```cpp
Heuristics 2: Euclidean
    h = std::sqrt(std::pow((node1_coord(0) - node2_coord(0)), 2 ) +
    std::pow((node1_coord(1) - node2_coord(1)), 2 ) +
    std::pow((node1_coord(2) - node2_coord(2)), 2 ));
```

（3）对角线

 算法实现

```cpp
Heuristics 3: Diagnol distance
double dx = std::abs(node1_coord(0) - node2_coord(0) );
double dy = std::abs(node1_coord(1) - node2_coord(1) );
double dz = std::abs(node1_coord(2) - node2_coord(2) );
double min_xyz = std::min({dx, dy, dz});
h = dx + dy + dz + (std::sqrt(3.0) -3) * min_xyz;
```

在经过实验后，综合计算时间和路径长度来看。欧几里得比其他启发函数略胜一筹，所以在Astar_searcher.cpp文件中选择启发函数2欧几里得距离

 

## 3、未来优化

1. 将随机地图固定
2. 在同一地图中同时规划出两条或多条路径，以便观察哪个启发函数的更快，规划的路径更短。

## 4、总结

在选择哪一个启发函数时，我们小组经历了一个漫长的过程。在开始时，我们先选择了在曼哈顿距离的基础上进行优化，后来发现欧几里得距离优势可能更大，经过多次实验并对比之后，确认欧几里距离优化后得更胜一筹



## 5、参考文献

（1）《A* Pathfinding For Beginners》https://www.gamedev.net/reference/articles/article2003.asp

（2)《基于节点优化的A*算法路径规划》

https://kns.cnki.net/kcms/detail/detail.aspx?dbcode=CJFD&dbname=CJFDLAST2021&filename=TSSF202103021&uniplatform=NZKPT&v=1zKATsjb9WhfVH1tLcHlrG56cDXspgbcCxQ3SrmRD-8LnoKdcbHwHRUrIAdVZpBi



# 建立二维栅格化地图

 [TOC]

 

## **1**环境搭建

Ubuntu 18.04 

ROS melodic

Cartographer

## **2**实践过程

因为我们手上没有雷达，所以我们使用Cartographer官网提供的2D和3D地图包进行建图测试

首先我们下载并启动2D地图包演示，代码如下

```
wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_2d/cartographer_paper_deutsches_museum.bag
```

```
roslaunch cartographer_ros demo_backpack_2d.launch bag_filename:=*${*HOME*}*/Downloads/cartographer_paper_deutsches_museum.bag
```

 

操作视频链接：【使用cartographer对德意志博物馆slam建图测试-哔哩哔哩】 https://b23.tv/mVZwXFM

 

然后我们下载并启动3D地图包演示，代码如下

```
wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_3d/with_intensities/b3-2016-04-05-14-14-00.bag
```

```
roslaunch cartographer_ros demo_backpack_3d.launch bag_filename:=*${*HOME*}*/Downloads/b3-2016-04-05-14-14-00.bag
```

 

操作视频链接：【使用cartographer进行三维slam建图-哔哩哔哩】 https://b23.tv/VVByu1C

 

## **3**总结

这次的Cartographer建图测试过程中我们遇到了很多问题，比如Cartographe的安装我们安装了多次，但都有着不同的问题，最后按照官方给出的安装教程才安装成功。还有就是这个二维和三维建图的过程，我们按照官方给出的链接进行操作，才省去了很大的麻烦。



# cartographer在arm架构系统下安装

## 1.安装依赖包

```bash
# Install the required libraries that are available as debs.
sudo apt-get update
sudo apt-get install -y \
    cmake \
    g++ \
    git \
    google-mock \
    libboost-all-dev \
    libcairo2-dev \
    libeigen3-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    liblua5.2-dev \
    libprotobuf-dev \
    libsuitesparse-dev \
    libwebp-dev \
    ninja-build \
    protobuf-compiler \
    python-sphinx
```

```
python-sphinx   可以换成 sphinx-common
```

## 2.安装ceres solver

```bash
cd  ~/Documents
git clone https://github.com.cnpmjs.org/ceres-solver/ceres-solver.git
cd ceres-solver
git checkout 1.14.x
mkdir build
cd build
cmake ..  #git hook时间比较长，耐心等待
make -j
sudo make install
```

## 3.安装arm架构下的prtobuf 3.6.1

##### 准备

` sudo apt install autoconf automake libtool curl make g++ unzip git`

安装arm工具链

`sudo apt install gcc-arm-linux-gnueabihf`

`sudo apt install g++-arm-linux-gnueabihf`

##### 下载

下载 protobuf 和对应版本的 protoc

`wget https://github.com/protocolbuffers/protobuf/releases/download/v3.6.1/protobuf-cpp-3.6.1.tar.gz`
`wget https://github.com/protocolbuffers/protobuf/releases/download/v3.6.1/protoc-3.6.1-linux-aarch_64.zip`


##### 解压

```
tar xf protobuf-cpp-3.6.1.tar.gz
mkdir protoc-3.6.1
unzip protoc-3.6.1-linux-aarch_64.zip -d protoc-3.6.1
```



##### 编译

```
cd protobuf-3.6.1
./configure 
make
make check 
sudo make install 
```

## 4．安装cartographer

```bash
cd  ~/Documents
git clone https://gitee.com/BlueWhaleRobot/cartographer.git
cd cartographer
mkdir build
cd build
cmake ..
make -j12
sudo make install
```

#### 如遇到abseil为安装则

`sudo apt-get install stow`

`sudo chmod +x ~/Documents/cartographer/scripts/install_abseil.sh`

`cd ~/cartographer_ws/src/cartographer/scripts`

`./install_abseil.sh`

## 5．安装cartographer_ros

```bash
cd ~/Documents/ros/src   #请修改路径到自己的ROS catkin工作空间
git clone https://gitee.com/BlueWhaleRobot/cartographer_ros.git
cd ..
catkin_make
```