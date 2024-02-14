#include "Astar_searcher.h"

using namespace std;
using namespace Eigen;
// 初始化地图
void AstarPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id)
{   // gl_x表示地图边界,l->low(下边界),u->up(上边界)
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);
    
    GLX_SIZE = max_x_id;	
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;	
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;	

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution; 
	// void *memset(void *s, int ch, size_t n);将s中当前位置后面的n个字节(typedef unsigned int size_t)用ch替换并返回 s 。
    data = new uint8_t[GLXYZ_SIZE];	
	
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
    
    GridNodeMap = new GridNodePtr ** [GLX_SIZE];
        for(int i = 0; i < GLX_SIZE; i++){
        GridNodeMap[i] = new GridNodePtr * [GLY_SIZE]; 
        for(int j = 0; j < GLY_SIZE; j++){
            GridNodeMap[i][j] = new GridNodePtr [GLZ_SIZE];
            for( int k = 0; k < GLZ_SIZE;k++){
                Vector3i tmpIdx(i,j,k);	
                Vector3d pos = gridIndex2coord(tmpIdx);
                // GridNodeMap-三维指针;传入网格地图坐标和实际坐标,初始化每个节点的属性
                GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos); 
            }
        }
    }
}
// 将所有点的属性设置为未访问过的状态下
void AstarPathFinder::resetGrid(GridNodePtr ptr)
{
    ptr->id = 0;
    ptr->cameFrom = NULL;
    ptr->gScore = inf;
    ptr->fScore = inf;
}

void AstarPathFinder::resetUsedGrids()
{   
    for(int i=0; i < GLX_SIZE ; i++)
        for(int j=0; j < GLY_SIZE ; j++)
            for(int k=0; k < GLZ_SIZE ; k++)
                resetGrid(GridNodeMap[i][j][k]);
}

void AstarPathFinder::setObs(const double coord_x, const double coord_y, const double coord_z)
{   
    if( coord_x < gl_xl  || coord_y < gl_yl  || coord_z <  gl_zl || 
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu )
        return;

    int idx_x = static_cast<int>( (coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>( (coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>( (coord_z - gl_zl) * inv_resolution);      
	
    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
}


vector<Vector3d> AstarPathFinder::getVisitedNodes()
{   
    vector<Vector3d> visited_nodes;  
    for(int i = 0; i < GLX_SIZE; i++)
        for(int j = 0; j < GLY_SIZE; j++)
            for(int k = 0; k < GLZ_SIZE; k++){                   
                if(GridNodeMap[i][j][k]->id == -1) 
                    visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
            }

    ROS_WARN("visited_nodes size : %d", visited_nodes.size());
    return visited_nodes;
}

Vector3d AstarPathFinder::gridIndex2coord(const Vector3i & index) 
{
    Vector3d pt;    

    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

    return pt;
}


Vector3i AstarPathFinder::coord2gridIndex(const Vector3d & pt) 
{
    Vector3i idx;
    idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
            min( max( int( (pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);                  
  
    return idx;
}

Eigen::Vector3d AstarPathFinder::coordRounding(const Eigen::Vector3d & coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}

inline bool AstarPathFinder::isOccupied(const Eigen::Vector3i & index) const
{
    return isOccupied(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isFree(const Eigen::Vector3i & index) const
{
    return isFree(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return  (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

inline bool AstarPathFinder::isFree(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
           (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

/*
        函数AstarGetSucc（…）为拓展节点函数
        a.函数有3个输入参数，GridNodePtr currentPtr：当前节点指针，其具体内容见node.h文档中结构体GridNode内容；
        vector<GridNodePtr> & neighborPtrSets：是一个容器，存放当前节点扩展的周围邻居节点的指针；
        vector<double> & edgeCostSets：是一个容器，存放当前节点到周围邻居节点的代价。
        b.当前节点遍历邻居节点的过程：将当前节点的栅格坐标赋值给this_x,this_y,this_z,然后进行三次循环，
        循环的起点是-1，终点是1，每次循环也就有了三个索引-1，0，1。
        栅格坐标和for循环的三个索引相加就会上下左右偏移位置，也就可以遍历周围的节点。
        在循环内部需要加2个限制条件： 1、如果拓展的节点不满足地图坐标并超出了边界就跳过，
        2、如果拓展的节点是占据状态就跳过。
        如果满足这两个条件就可以把这个节点push_back进容器neighborPtrSets中。
        还有一个存放代价的容edgeCostSets，需要把遍历到的邻居节点的栅格坐标转化为真实坐标后与当前节点进行欧式距离进行求解，
        push_back进edgeCostSets即可。
*/
       
inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> & neighborPtrSets, vector<double> & edgeCostSets)
{   
    neighborPtrSets.clear();  
    edgeCostSets.clear();
    	
	if(currentPtr == nullptr)
    std::cout << "Error: Current pointer is null!" << endl;

//取出当前节点栅格地图坐标
    Eigen::Vector3i thisNode = currentPtr -> index;
    int this_x = thisNode[0];
    int this_y = thisNode[1];
    int this_z = thisNode[2];
    auto this_coord = currentPtr -> coord;
    int  n_x, n_y, n_z;
    double dist;
	
    GridNodePtr temp_ptr = nullptr;
    Eigen::Vector3d n_coord;
    
//遍历周围点,获取周围点的edgeCostSets
    for(int i = -1;i <= 1;++i ){
        for(int j = -1;j <= 1;++j ){
            for(int k = -1;k <= 1;++k){
                if( i == 0 && j == 0 && k == 0)
                    continue; 

                n_x = this_x + i;
                n_y = this_y + j;
                n_z = this_z + k;

                if( (n_x < 0) || (n_x > (GLX_SIZE - 1)) || (n_y < 0) || (n_y > (GLY_SIZE - 1) ) || (n_z < 0) || (n_z > (GLZ_SIZE - 1)))
                    continue; 

                if(isOccupied(n_x, n_y, n_z))
                    continue; 
                
                temp_ptr = GridNodeMap[n_x][n_y][n_z];

                if(temp_ptr->id == -1) 
                    continue; 

                n_coord = temp_ptr->coord;

                if(temp_ptr == currentPtr){
                    std::cout << "Error: temp_ptr == currentPtr)" << std::endl;
                }

                if( (std::abs(n_coord[0] - this_coord[0]) < 1e-6) and (std::abs(n_coord[1] - this_coord[1]) < 1e-6) and (std::abs(n_coord[2] - this_coord[2]) < 1e-6 )){
                    std::cout << "Error: Not expanding correctly!" << std::endl;
                    std::cout << "n_coord:" << n_coord[0] << " "<<n_coord[1]<<" "<<n_coord[2] << std::endl;
                    std::cout << "this_coord:" << this_coord[0] << " "<<this_coord[1]<<" "<<this_coord[2] << std::endl;

                    std::cout << "current node index:" << this_x << " "<< this_y<<" "<< this_z << std::endl;
                    std::cout << "neighbor node index:" << n_x << " "<< n_y<<" "<< n_z << std::endl;
                }

                dist = std::sqrt( (n_coord[0] - this_coord[0]) * (n_coord[0] - this_coord[0])+
                        (n_coord[1] - this_coord[1]) * (n_coord[1] - this_coord[1])+
                        (n_coord[2] - this_coord[2]) * (n_coord[2] - this_coord[2]));
                
                neighborPtrSets.push_back(temp_ptr); 
                edgeCostSets.push_back(dist);

            }
        }
    }
}

double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2)
/*
函数getHeu（…）为启发函数
在此部分代码中，给出求取启发函数的3种基本方法，欧几里得Euclidean、曼哈顿Manhattan和对角线Diagnol distance，
你可以在此基础上进行改进或者选择其他更优方法求取。
代码如下：
*/
{
     double h;
    auto node1_coord = node1->coord;
    auto node2_coord = node2->coord;
// Heuristics 1: Manhattan
// h = std::abs(node1_coord(0) - node2_coord(0) ) +
//     std::abs(node1_coord(1) - node2_coord(1) ) +
//     std::abs(node1_coord(2) - node2_coord(2) );
Heuristics 2: Euclidean
    h = std::sqrt(std::pow((node1_coord(0) - node2_coord(0)), 2 ) +
    std::pow((node1_coord(1) - node2_coord(1)), 2 ) +
    std::pow((node1_coord(2) - node2_coord(2)), 2 ));

// Heuristics 3: Diagnol distance
//double dx = std::abs(node1_coord(0) - node2_coord(0) );
//double dy = std::abs(node1_coord(1) - node2_coord(1) );
//double dz = std::abs(node1_coord(2) - node2_coord(2) );
//double min_xyz = std::min({dx, dy, dz});
//h = dx + dy + dz + (std::sqrt(3.0) -3) * min_xyz;


    return 0;
}

/*
函数AstarGraphSearch(…)为A*寻路函数
主循环的内容如下：
a.将总的搜索代价f(n)最小的节点从openSet移到closeSet,并判断是否id=-1,若id=-1，节点属于closeSet,跳出此次循环；
b.判断此节点是否是终点，如果是跳出此次循环；
c.通过AstarGetSucc（…）函数获得当前节点的邻居节点指针和g(n)；
d.遍历节点所有未扩展的邻居,如果邻居节点的id=0，则是未被发现的节点，需要放到openSet中；
如果id=1,则已在openSet中，判断其是否需要更新，更新条件为：如果g(m)>g(n)+Cmn,g(m)=g(n)+Cmn
*/
void AstarPathFinder::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt)
{   
    ros::Time time_1 = ros::Time::now();    

//将rviz中第节点起始坐标和终点坐标变换为在grid图中的节点坐标
    Vector3i start_idx = coord2gridIndex(start_pt);
    Vector3i end_idx   = coord2gridIndex(end_pt);
    goalIdx = end_idx;

    start_pt = gridIndex2coord(start_idx);
    end_pt   = gridIndex2coord(end_idx);
  
//将起点和终点通过结构体构造函数进行初始化
    GridNodePtr startPtr = new GridNode(start_idx, start_pt);
    GridNodePtr endPtr   = new GridNode(end_idx,   end_pt);
//openSet是容器，存放当前节点扩展到的邻居节点的指针
    openSet.clear();
//将当前节点和邻居节点指针置空
    GridNodePtr currentPtr  = NULL;
    GridNodePtr neighborPtr = NULL;

//设置开始节点的g(n)和h(n)
    startPtr -> gScore = 0;
    startPtr -> fScore = getHeu(startPtr,endPtr);   
// 1--> open set, -1 --> closed set,0 --> 未被访问过
    startPtr -> id = 1; 
    startPtr -> coord = start_pt;
//将开始节点放入openSet容器中
    openSet.insert( make_pair(startPtr -> fScore, startPtr) ); 
  
    GridNodeMap[start_idx[0]][start_idx[1]][start_idx[2]] -> id = 1;

    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;
    Eigen::Vector3i current_idx; 
// 开始主循环
    while ( !openSet.empty() ){
// openset:待访问节点容器;closed set:访问过节点容器
        int x = openSet.begin()->second->index(0); 
        int y = openSet.begin()->second->index(1); 
        int z = openSet.begin()->second->index(2);
        openSet.erase(openSet.begin());
        currentPtr = GridNodeMap[x][y][z];
       
        if(currentPtr->id == -1)
            continue;
        
        currentPtr->id = -1;

       
        if( currentPtr->index == goalIdx )
        {
            ros::Time time_2 = ros::Time::now();
            terminatePtr = currentPtr;
            ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );            
            return;
        }
//通过AstarGetSucc获得当前节点的邻居节点指针和g(n)     
      
        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);     
        for(int i = 0; i < (int)neighborPtrSets.size(); i++){
            
            neighborPtr = neighborPtrSets[i];
            if(neighborPtr -> id == 0){ 
                
                neighborPtr->gScore = currentPtr->gScore + edgeCostSets[i];
                neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr,endPtr);
                neighborPtr->cameFrom = currentPtr; 
                
                openSet.insert(make_pair(neighborPtr -> fScore, neighborPtr));
                neighborPtr -> id = 1;
                continue;
            }
            else if(neighborPtr -> id == 1){ 
                if( neighborPtr->gScore > (currentPtr->gScore+edgeCostSets[i]))
                {
                    neighborPtr -> gScore = currentPtr -> gScore + edgeCostSets[i];
                    neighborPtr -> fScore = neighborPtr -> gScore + getHeu(neighborPtr,endPtr);
                    neighborPtr -> cameFrom = currentPtr;
                }

                continue;
            }
            else{
                continue;
            }
        }      
    }
   
    ros::Time time_2 = ros::Time::now();
    if((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in Astar path finding is %f", (time_2 - time_1).toSec() );
}


vector<Vector3d> AstarPathFinder::getPath() 
{   
    vector<Vector3d> path;
    vector<GridNodePtr> gridPath;
    
    auto ptr = terminatePtr;
    while(ptr -> cameFrom != NULL){
        gridPath.push_back(ptr);
        ptr = ptr->cameFrom;
    }

    for (auto ptr: gridPath)
        path.push_back(ptr->coord);
        
    reverse(path.begin(),path.end()); 	
    return path;
}
