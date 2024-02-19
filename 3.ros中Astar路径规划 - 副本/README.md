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
