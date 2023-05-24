#pragma once
#include<iostream>
#include<graphics.h>
#include<vector>
#include<string>
#include<ctime>
using namespace std;

constexpr auto ZXCOST = 10;//ÿ����ֱ�ߵĴ���
constexpr auto XXCOST = 14;//ÿ����б�ߵĴ���
constexpr auto MAPTYPE = 2;//��ͼ����
constexpr auto ROWS = 60;//��ͼ���� //60
constexpr auto COLS = 50;//��ͼ���� //50
constexpr auto Swidth = 1200;//1200
constexpr auto Sheight = 1400;//1400
constexpr auto ratio = 2;//��ͼ����

enum algoType//�㷨����
{
	Astar,//Astar�㷨
	Dijkstra,//Dijkstra�㷨
	BestFS//��������㷨
};

enum direct//����
{
	p_up,//��
	p_down,//��
	p_left,//��
	p_right,//��
	p_l_up,//����
	p_r_up,//����
	p_l_down,//����
	p_r_down//����
};

class Point//��ͼ�е���С��Ԫ��Ҳ���ǵ�
{
public:
	int row;//�ڵ�ͼ�е��У���������
	int col;//�ڵ�ͼ�е��У���������

	int f = 0;//�ܴ���
	int g = 0;//��㵽�õ�Ĵ��ۣ���֪
	int h = 0;//�õ㵽�յ�Ĵ��ۣ�����
};

class Map//��ͼ
{
public:
	Map();//��ʼ����ͼ
	void showMap();//��ӡ��ͼ
	void drawMap();//���Ƶ�ͼ
	void drawRec(int row, int col, COLORREF color = WHITE);//���������������������������λ�ã�Ȼ����ƾ���
	void delay(int time);//��ʱ

public:
	int myMap[ROWS][COLS] = { 0 };//��ͼ�ö�ά�����ʾ
	Point beginP = { 1, 1 };//{ ROWS - 2, 1 };//{1, 1};//���
	Point endP = { ROWS - 2,  COLS - 2 };//{6, 7};//�յ� //������ʼ���󣬺����f g h�ḳֵΪ0��

	int disX = (Swidth - (COLS - 1) * ZXCOST * ratio) / 2;//���ϵ��봰����ߵľ��룬�õ�ͼ����
	int disY = (Sheight - (ROWS - 1) * ZXCOST * ratio) / 2;//���ϵ��봰���ϱߵľ��룬�õ�ͼ����
};

class treeNode//�����Ľṹ�洢�����֮��Ĺ�ϵ�����������Ľ��
{
public:
	treeNode(int row, int col);//��ʼ�����

public:
	Point pos;
	vector<treeNode*> child;//ָ���ӵ�ָ�룬ÿ���������ж�����ӣ�����ֱ����������ʾ��
	treeNode* pParent = nullptr;//ָ�򸸽���ָ�룬ÿ�����ֻ��һ������㣻
};

class tree//�����Ľṹ�洢�����֮��Ĺ�ϵ��ÿ����ĺ��ӱ�ʾ�������ߵ��¸���
{
public:
	tree();
	~tree();
	void updateTree();//������
	void showResult();//��ӡ���
	bool bBeginPoint(treeNode* point);//�Ƿ������
	bool bEndPoint(treeNode* point);//�Ƿ����յ�

public:
	Map map0;//��ͼ����
	treeNode* pRoot = nullptr;//�����
	treeNode* pCurrent = nullptr;//��ǰ���
	treeNode* pNext = nullptr;//��ǰ�������ڽ��

	vector<treeNode*> openList;//��������Сf�����飬�洢�������ĵ��ָ�룻
	bool closeList[ROWS][COLS] = { false };//����߹��ĵ㣬��ʼ��Ϊû�߹�
	vector<treeNode*> result;//�洢��ӡ����

	//bool bEnd = false;//�Ƿ񵽴��յ�
	int cost = 0;//�ۼƴ���
	int costFinal = 0;//����·�����ܴ���
	int type = Astar;//�㷨����
};

class ShortestPath //���·��
{
public:
	void shortestPath();

public:
	tree tr;
};