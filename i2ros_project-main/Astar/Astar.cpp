#include"Astar.h"

Map::Map()//初始化地图
{
	/*for (int i = 0; i < ROWS; i++)
	{
		for (int j = 0; j < COLS; j++)
		{
			myMap[i][j] = 0;
		}
	}*/

	//设置障碍物
	if (MAPTYPE == 0)
	{
		for (int i = 0; i < ROWS; i++)
		{
			for (int j = 0; j < COLS; j++)
			{
				if (j == 20 && i <= 40)
				{
					myMap[i][j] = 1;
				}
			}
		}
	}

	if (MAPTYPE == 1)
	{
		for (int i = 0; i < ROWS; i++)
		{
			for (int j = 0; j < COLS; j++)
			{
				if (j == 4 && i != 6)
				{
					myMap[i][j] = 1;
				}

				if (j == 12 && i != 20)
				{
					myMap[i][j] = 1;
				}

				if (j == 20 && i != 40)
				{
					myMap[i][j] = 1;
				}

				if (j == 35 && i != 20)
				{
					myMap[i][j] = 1;
				}

				if (i == 4 && j >= 2 && j <= 4)
				{
					myMap[i][j] = 1;
				}

				if (i == 3 && j >= 1 && j <= 2)
				{
					myMap[i][j] = 1;
				}

				if (i == ROWS - 10 && j >= COLS - 10 && j < COLS)
				{
					myMap[i][j] = 1;
				}
			}
		}
	}
	else if (MAPTYPE == 2)
	{
		for (int i = 0; i < ROWS; i++)
		{
			for (int j = 0; j < COLS; j++)
			{
				if (j == 3 && i >= 0 && i <= 10)
				{
					myMap[i][j] = 1;
				}

				if (j == 5 && i != 7 && i != 6)
				{
					myMap[i][j] = 1;
				}

				if (j == 11 && i >= 0 && i <= 25)
				{
					myMap[i][j] = 1;
				}

				if (j == COLS - 9 && i >= ROWS - 16)
				{
					myMap[i][j] = 1;
				}

				if (i == 40 && j >= 13 && j < COLS)
				{
					myMap[i][j] = 1;
				}

				if (i == 30 && j >= 5 && j <= 40)
				{
					myMap[i][j] = 1;
				}
			}
		}
	}
	else if (MAPTYPE == 3)
	{
		for (int i = 0; i < ROWS; i++)
		{
			for (int j = 0; j < COLS; j++)
			{
				if (j == 10 && (i >= 0 && /*i <= 20 || i >= 23 &&*/ i <= 50))
				{
					myMap[i][j] = 1;
				}

				if (j == 30 && (i >= 10 && /*i <= 50 || i >= 53 &&*/ i < ROWS))
				{
					myMap[i][j] = 1;
				}
			}
		}
	}

	showMap();
	drawMap();
}

void Map::showMap()//打印地图
{
	cout << "起点位置：(" << beginP.row << ", " << beginP.col << "); 终点位置：(" << endP.row << ", " << endP.col << "), 地图如下：" << endl;

	myMap[beginP.row][beginP.col] = 2;//起点
	myMap[endP.row][endP.col] = 9;//终点

	for (int i = 0; i < ROWS; i++)
	{
		for (int j = 0; j < COLS; j++)
		{
			cout << myMap[i][j] << " ";
		}
		cout << endl;
	}
	cout << endl;
}

void Map::drawMap()//绘制地图
{
	setbkcolor(WHITE);    //设置背景颜色
	setlinecolor(BLACK);    //设置边框颜色
	setlinestyle(PS_SOLID, 2);	// 设置线的样式为实线2px
	cleardevice();

	for (int i = 0; i < ROWS; i++)
	{
		for (int j = 0; j < COLS; j++)
		{
			if (myMap[i][j] == 1)
			{
				drawRec(i, j, BROWN);//障碍物褐色
			}
			else if (i == beginP.row && j == beginP.col)
			{
				drawRec(i, j, RED);//起点红色
			}
			else if (i == endP.row && j == endP.col)
			{
				drawRec(i, j, BLUE);//终点蓝色
			}
			else
			{
				drawRec(i, j);//默认白色
			}
		}
	}
}

void Map::drawRec(int row, int col, COLORREF color)//根据行数和列数，求矩形四条边位置，然后绘制矩形
{
	setfillcolor(color);//设置填充颜色

	int pointX = disX + col * ZXCOST * ratio;//X对应列
	int pointY = disY + row * ZXCOST * ratio;//Y对应行

	int left = pointX - ZXCOST / 2 * ratio;
	int top = pointY - ZXCOST / 2 * ratio;
	int right = left + ZXCOST * ratio;
	int bottom = top + ZXCOST * ratio;

	fillrectangle(left, top, right, bottom);
}

void Map::delay(int time) //延时函数，单位ms
{
	clock_t  now = clock();
	while (clock() - now < time)
	{

	}
}

treeNode::treeNode(int row, int col)//初始化结点
{
	pos.row = row;
	pos.col = col;
}

tree::tree()//初始化树
{
	pRoot = new treeNode(map0.beginP.row, map0.beginP.col);//起点放入树中根结点位置
	closeList[map0.beginP.row][map0.beginP.col] = true;//起点初始化为走过
	cout << "根结点：(" << pRoot->pos.row << ", " << pRoot->pos.col << ")" << endl;

	pCurrent = pRoot;//初始化当前结点为根结点
}

tree::~tree()
{
	if (pRoot != nullptr)
	{
		delete pRoot;
		pRoot = nullptr;
	}

	if (pNext != nullptr)
	{
		delete pNext;
		pNext = nullptr;
	}
}

void tree::updateTree()//更新树
{
	while (true)//整个寻路过程
	{
		BeginBatchDraw();
		//openList.clear();//每轮不需要清空，因为后面会erase()；

		for (int i = 0; i < 8; i++)//循环遍历每个点周围的8个点
		{
			pNext = new treeNode(pCurrent->pos.row, pCurrent->pos.col);//下个点初始化为当前点，因为还不知道往哪个方向走；
			//这个要放在for里，不能放在构造函数里，因为每次要从中心点开始走！			
			//不能写成pNext = pCurrent;
			//cout << "pCurrent: " << pNext->pos.row << ", " << pNext->pos.col << endl;

			switch (i)
			{
			case p_up:
			{
				pNext->pos.row--;
				pNext->pos.g += ZXCOST;
				break;
			}
			case p_down:
			{
				pNext->pos.row++;
				pNext->pos.g += ZXCOST;
				break;
			}
			case p_left:
			{
				pNext->pos.col--;
				pNext->pos.g += ZXCOST;
				break;
			}
			case p_right:
			{
				pNext->pos.col++;
				pNext->pos.g += ZXCOST;
				break;
			}
			case p_l_up:
			{
				pNext->pos.row--;
				pNext->pos.col--;
				pNext->pos.g += XXCOST;
				break;
			}
			case p_r_up:
			{
				pNext->pos.row--;
				pNext->pos.col++;
				pNext->pos.g += XXCOST;
				break;
			}
			case p_l_down:
			{
				pNext->pos.row++;
				pNext->pos.col--;
				pNext->pos.g += XXCOST;
				break;
			}
			case p_r_down:
			{
				pNext->pos.row++;
				pNext->pos.col++;
				pNext->pos.g += XXCOST;
				break;
			}
			default:
				break;
			}

			//cout << "pNext: " << pNext->pos.row << ", " << pNext->pos.col << endl;
			//system("pause");

			if (pNext->pos.row < 0 || pNext->pos.row >= ROWS || pNext->pos.col < 0 || pNext->pos.col >= COLS)
			{
				cout << "超出地图边界" << endl;
				continue;
			}

			//cout << closeList[pNext->pos.row][pNext->pos.col] << ", pNxet: " << pNext->pos.row << ", " << pNext->pos.col << endl;
			if (map0.myMap[pNext->pos.row][pNext->pos.col] == 1 || closeList[pNext->pos.row][pNext->pos.col]) //是障碍物或者走过
			{
				continue;
			}

			//计算h值
			int X = abs(pNext->pos.row - map0.endP.row);
			int Y = abs(pNext->pos.col - map0.endP.col);
			pNext->pos.h = (X + Y) * ZXCOST;//曼哈顿距离

			//计算f值
			if (type == Astar)
			{
				pNext->pos.f = cost + pNext->pos.h;//Astar算法，这里g只能表示单步的代价值，而cost才是从起点到该点的累计代价值
			}
			else if (type == Dijkstra)
			{
				pNext->pos.f = cost;//Dijkstra算法
			}
			else if (type == BestFS)
			{
				pNext->pos.f = pNext->pos.h;//最佳优先算法
			}

			pCurrent->child.push_back(pNext);//存入树中，作为当前点的孩子
			pNext->pParent = pCurrent;//下个点的父结点指向当前点

			bool flag = false;
			for (auto iter = openList.begin(); iter != openList.end(); iter++)
			{
				if ((*iter)->pos.row == pNext->pos.row && (*iter)->pos.col == pNext->pos.col)
				{
					flag = true;//防止重复存入相同的点
					break;
				}
			}
			if (!flag)
			{
				openList.push_back(pNext);//存入openList数组中，用于寻找最小f的点
			}

			//cout << "pNext: " << pNext->pos.row << ", " << pNext->pos.col << endl;
			//system("pause");
		}

		/*for (auto iter = openList.begin(); iter != openList.end(); iter++)
		{
			cout << (*iter)->pos.row << ", " << (*iter)->pos.col << endl;
		}*/
		//system("pause");

		//从openList数组中选取f最小的点
		auto itMin = openList.begin();
		for (auto it = openList.begin(); it != openList.end(); it++)
		{
			if ((*it)->pos.f < (*itMin)->pos.f)
			{
				itMin = it;
			}
		}

		pCurrent = *itMin;//走到这个点
		closeList[pCurrent->pos.row][pCurrent->pos.col] = true;//标记为走过
		map0.myMap[pCurrent->pos.row][pCurrent->pos.col] = 5;//显示为5
		cost += (*itMin)->pos.g;//中间试探点的累计路径长度
		cout << "f值最小的点是：(" << (*itMin)->pos.row << ", " << (*itMin)->pos.col << "), f=" << (*itMin)->pos.f << ", g=" << (*itMin)->pos.g << ", h=" << (*itMin)->pos.h << ", 累计代价是：" << cost << endl;
		itMin = openList.erase(itMin);//从openList数组里删除这个点

		/*for (auto iter = openList.begin(); iter != openList.end(); iter++)
		{
			cout << (*iter)->pos.row << ", " << (*iter)->pos.col << endl;
		}*/
		//system("pause");

		/*for (int i = 0; i < ROWS; i++)
		{
			for (int j = 0; j < COLS; j++)
			{
				cout << closeList[i][j] << " ";
			}
			cout << endl;
		}
		cout << endl;*/

		//判断是否找到终点
		if (bEndPoint(pCurrent))
		{
			cout << "已到达终点" << endl << endl;
			showResult();
			break;//到达终点，退出while循环
		}
		else
		{
			map0.drawRec(pCurrent->pos.row, pCurrent->pos.col, YELLOW);//用黄色填充试探的路径点
		}

		if (openList.empty())//判断是否走完了整个地图
		{
			cout << "没有找到终点" << endl << endl;
			break;//走完了整个地图都没找到终点，也退出while循环
		}

		EndBatchDraw();
		//map0.delay(100);//用于实时展示
	}
}

void tree::showResult()//打印结果
{
	while (pCurrent)//循环遍历树，直到pCurrent为空，也就是到根结点为止
	{
		BeginBatchDraw();
		//cout << "(" << pCurrent->pos.row << ", " << pCurrent->pos.col << ") ";//这样直接打印是逆序的，从终点到起点；
		result.insert(result.begin(), pCurrent);//头插存入容器，不然只能逆序打印
		costFinal += pCurrent->pos.g;//累计路径长度

		if (!bBeginPoint(pCurrent) && !bEndPoint(pCurrent))//如果不是起点和终点
		{
			map0.myMap[pCurrent->pos.row][pCurrent->pos.col] = 6;//用6标记最终的路径点
			map0.drawRec(pCurrent->pos.row, pCurrent->pos.col, GREEN);//用绿色填充最终的路径点
		}

		pCurrent = pCurrent->pParent;//指针向上移动到父结点
		EndBatchDraw();
	}
	map0.showMap();

	string typeName;
	if (type == Astar)
	{
		typeName = "Astar";
	}
	else if (type == Dijkstra)
	{
		typeName = "Dijkstra";
	}
	else if (type == BestFS)
	{
		typeName = "BestFS";
	}
	cout << typeName << "算法求得的最短路径是：" << endl;
	for (auto it = result.begin(); it != result.end(); it++)
	{
		cout << "(" << (*it)->pos.row << ", " << (*it)->pos.col << ")";
		if (!bEndPoint(*it))//不是终点就打印箭头
		{
			cout << " -> ";
		}
	}
	cout << endl;

	cout << "总路径长度是：" << costFinal << endl;
}

bool tree::bBeginPoint(treeNode* point)//是否是起点
{
	if (point->pos.row == map0.beginP.row && point->pos.col == map0.beginP.col)
	{
		return true;
	}
	return false;
}

bool tree::bEndPoint(treeNode* point)//是否是终点
{
	if (point->pos.row == map0.endP.row && point->pos.col == map0.endP.col)
	{
		return true;
	}
	return false;
}

void ShortestPath::shortestPath()//求解路径
{
	system("pause");
	tr.updateTree();
}





int main()
{
	initgraph(Swidth, Sheight, EW_SHOWCONSOLE);// easyx初始化，同时创建图形窗口和命令窗口	

	ShortestPath sPath;
	sPath.shortestPath();

	system("pause");
	closegraph();
	return 0;
}