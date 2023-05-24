#pragma once
#include<iostream>
#include<graphics.h>
#include<vector>
#include<string>
#include<ctime>
using namespace std;

constexpr auto ZXCOST = 10;//每步走直线的代价
constexpr auto XXCOST = 14;//每步走斜线的代价
constexpr auto MAPTYPE = 2;//地图类型
constexpr auto ROWS = 60;//地图行数 //60
constexpr auto COLS = 50;//地图列数 //50
constexpr auto Swidth = 1200;//1200
constexpr auto Sheight = 1400;//1400
constexpr auto ratio = 2;//绘图比例

enum algoType//算法类型
{
	Astar,//Astar算法
	Dijkstra,//Dijkstra算法
	BestFS//最佳优先算法
};

enum direct//方向
{
	p_up,//上
	p_down,//下
	p_left,//左
	p_right,//右
	p_l_up,//左上
	p_r_up,//右上
	p_l_down,//左下
	p_r_down//右下
};

class Point//地图中的最小单元，也就是点
{
public:
	int row;//在地图中的行，不是坐标
	int col;//在地图中的列，不是坐标

	int f = 0;//总代价
	int g = 0;//起点到该点的代价，已知
	int h = 0;//该点到终点的代价，估算
};

class Map//地图
{
public:
	Map();//初始化地图
	void showMap();//打印地图
	void drawMap();//绘制地图
	void drawRec(int row, int col, COLORREF color = WHITE);//根据行数和列数，求矩形四条边位置，然后绘制矩形
	void delay(int time);//延时

public:
	int myMap[ROWS][COLS] = { 0 };//地图用二维数组表示
	Point beginP = { 1, 1 };//{ ROWS - 2, 1 };//{1, 1};//起点
	Point endP = { ROWS - 2,  COLS - 2 };//{6, 7};//终点 //这样初始化后，后面的f g h会赋值为0；

	int disX = (Swidth - (COLS - 1) * ZXCOST * ratio) / 2;//左上点离窗口左边的距离，让地图居中
	int disY = (Sheight - (ROWS - 1) * ZXCOST * ratio) / 2;//左上点离窗口上边的距离，让地图居中
};

class treeNode//用树的结构存储点与点之间的关系，这里是树的结点
{
public:
	treeNode(int row, int col);//初始化结点

public:
	Point pos;
	vector<treeNode*> child;//指向孩子的指针，每个结点可以有多个孩子，所以直接用容器表示；
	treeNode* pParent = nullptr;//指向父结点的指针，每个结点只有一个父结点；
};

class tree//用树的结构存储点与点之间的关系：每个点的孩子表示这个点可走的下个点
{
public:
	tree();
	~tree();
	void updateTree();//更新树
	void showResult();//打印结果
	bool bBeginPoint(treeNode* point);//是否是起点
	bool bEndPoint(treeNode* point);//是否是终点

public:
	Map map0;//地图对象
	treeNode* pRoot = nullptr;//根结点
	treeNode* pCurrent = nullptr;//当前结点
	treeNode* pNext = nullptr;//当前结点的相邻结点

	vector<treeNode*> openList;//用来找最小f的数组，存储遍历过的点的指针；
	bool closeList[ROWS][COLS] = { false };//标记走过的点，初始化为没走过
	vector<treeNode*> result;//存储打印数据

	//bool bEnd = false;//是否到达终点
	int cost = 0;//累计代价
	int costFinal = 0;//最终路径的总代价
	int type = Astar;//算法类型
};

class ShortestPath //最短路径
{
public:
	void shortestPath();

public:
	tree tr;
};