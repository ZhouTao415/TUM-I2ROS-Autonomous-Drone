#include"Astar.h"

Map::Map()//��ʼ����ͼ
{
	/*for (int i = 0; i < ROWS; i++)
	{
		for (int j = 0; j < COLS; j++)
		{
			myMap[i][j] = 0;
		}
	}*/

	//�����ϰ���
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

void Map::showMap()//��ӡ��ͼ
{
	cout << "���λ�ã�(" << beginP.row << ", " << beginP.col << "); �յ�λ�ã�(" << endP.row << ", " << endP.col << "), ��ͼ���£�" << endl;

	myMap[beginP.row][beginP.col] = 2;//���
	myMap[endP.row][endP.col] = 9;//�յ�

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

void Map::drawMap()//���Ƶ�ͼ
{
	setbkcolor(WHITE);    //���ñ�����ɫ
	setlinecolor(BLACK);    //���ñ߿���ɫ
	setlinestyle(PS_SOLID, 2);	// �����ߵ���ʽΪʵ��2px
	cleardevice();

	for (int i = 0; i < ROWS; i++)
	{
		for (int j = 0; j < COLS; j++)
		{
			if (myMap[i][j] == 1)
			{
				drawRec(i, j, BROWN);//�ϰ����ɫ
			}
			else if (i == beginP.row && j == beginP.col)
			{
				drawRec(i, j, RED);//����ɫ
			}
			else if (i == endP.row && j == endP.col)
			{
				drawRec(i, j, BLUE);//�յ���ɫ
			}
			else
			{
				drawRec(i, j);//Ĭ�ϰ�ɫ
			}
		}
	}
}

void Map::drawRec(int row, int col, COLORREF color)//���������������������������λ�ã�Ȼ����ƾ���
{
	setfillcolor(color);//���������ɫ

	int pointX = disX + col * ZXCOST * ratio;//X��Ӧ��
	int pointY = disY + row * ZXCOST * ratio;//Y��Ӧ��

	int left = pointX - ZXCOST / 2 * ratio;
	int top = pointY - ZXCOST / 2 * ratio;
	int right = left + ZXCOST * ratio;
	int bottom = top + ZXCOST * ratio;

	fillrectangle(left, top, right, bottom);
}

void Map::delay(int time) //��ʱ��������λms
{
	clock_t  now = clock();
	while (clock() - now < time)
	{

	}
}

treeNode::treeNode(int row, int col)//��ʼ�����
{
	pos.row = row;
	pos.col = col;
}

tree::tree()//��ʼ����
{
	pRoot = new treeNode(map0.beginP.row, map0.beginP.col);//���������и����λ��
	closeList[map0.beginP.row][map0.beginP.col] = true;//����ʼ��Ϊ�߹�
	cout << "����㣺(" << pRoot->pos.row << ", " << pRoot->pos.col << ")" << endl;

	pCurrent = pRoot;//��ʼ����ǰ���Ϊ�����
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

void tree::updateTree()//������
{
	while (true)//����Ѱ·����
	{
		BeginBatchDraw();
		//openList.clear();//ÿ�ֲ���Ҫ��գ���Ϊ�����erase()��

		for (int i = 0; i < 8; i++)//ѭ������ÿ������Χ��8����
		{
			pNext = new treeNode(pCurrent->pos.row, pCurrent->pos.col);//�¸����ʼ��Ϊ��ǰ�㣬��Ϊ����֪�����ĸ������ߣ�
			//���Ҫ����for����ܷ��ڹ��캯�����Ϊÿ��Ҫ�����ĵ㿪ʼ�ߣ�			
			//����д��pNext = pCurrent;
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
				cout << "������ͼ�߽�" << endl;
				continue;
			}

			//cout << closeList[pNext->pos.row][pNext->pos.col] << ", pNxet: " << pNext->pos.row << ", " << pNext->pos.col << endl;
			if (map0.myMap[pNext->pos.row][pNext->pos.col] == 1 || closeList[pNext->pos.row][pNext->pos.col]) //���ϰ�������߹�
			{
				continue;
			}

			//����hֵ
			int X = abs(pNext->pos.row - map0.endP.row);
			int Y = abs(pNext->pos.col - map0.endP.col);
			pNext->pos.h = (X + Y) * ZXCOST;//�����پ���

			//����fֵ
			if (type == Astar)
			{
				pNext->pos.f = cost + pNext->pos.h;//Astar�㷨������gֻ�ܱ�ʾ�����Ĵ���ֵ����cost���Ǵ���㵽�õ���ۼƴ���ֵ
			}
			else if (type == Dijkstra)
			{
				pNext->pos.f = cost;//Dijkstra�㷨
			}
			else if (type == BestFS)
			{
				pNext->pos.f = pNext->pos.h;//��������㷨
			}

			pCurrent->child.push_back(pNext);//�������У���Ϊ��ǰ��ĺ���
			pNext->pParent = pCurrent;//�¸���ĸ����ָ��ǰ��

			bool flag = false;
			for (auto iter = openList.begin(); iter != openList.end(); iter++)
			{
				if ((*iter)->pos.row == pNext->pos.row && (*iter)->pos.col == pNext->pos.col)
				{
					flag = true;//��ֹ�ظ�������ͬ�ĵ�
					break;
				}
			}
			if (!flag)
			{
				openList.push_back(pNext);//����openList�����У�����Ѱ����Сf�ĵ�
			}

			//cout << "pNext: " << pNext->pos.row << ", " << pNext->pos.col << endl;
			//system("pause");
		}

		/*for (auto iter = openList.begin(); iter != openList.end(); iter++)
		{
			cout << (*iter)->pos.row << ", " << (*iter)->pos.col << endl;
		}*/
		//system("pause");

		//��openList������ѡȡf��С�ĵ�
		auto itMin = openList.begin();
		for (auto it = openList.begin(); it != openList.end(); it++)
		{
			if ((*it)->pos.f < (*itMin)->pos.f)
			{
				itMin = it;
			}
		}

		pCurrent = *itMin;//�ߵ������
		closeList[pCurrent->pos.row][pCurrent->pos.col] = true;//���Ϊ�߹�
		map0.myMap[pCurrent->pos.row][pCurrent->pos.col] = 5;//��ʾΪ5
		cost += (*itMin)->pos.g;//�м���̽����ۼ�·������
		cout << "fֵ��С�ĵ��ǣ�(" << (*itMin)->pos.row << ", " << (*itMin)->pos.col << "), f=" << (*itMin)->pos.f << ", g=" << (*itMin)->pos.g << ", h=" << (*itMin)->pos.h << ", �ۼƴ����ǣ�" << cost << endl;
		itMin = openList.erase(itMin);//��openList������ɾ�������

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

		//�ж��Ƿ��ҵ��յ�
		if (bEndPoint(pCurrent))
		{
			cout << "�ѵ����յ�" << endl << endl;
			showResult();
			break;//�����յ㣬�˳�whileѭ��
		}
		else
		{
			map0.drawRec(pCurrent->pos.row, pCurrent->pos.col, YELLOW);//�û�ɫ�����̽��·����
		}

		if (openList.empty())//�ж��Ƿ�������������ͼ
		{
			cout << "û���ҵ��յ�" << endl << endl;
			break;//������������ͼ��û�ҵ��յ㣬Ҳ�˳�whileѭ��
		}

		EndBatchDraw();
		//map0.delay(100);//����ʵʱչʾ
	}
}

void tree::showResult()//��ӡ���
{
	while (pCurrent)//ѭ����������ֱ��pCurrentΪ�գ�Ҳ���ǵ������Ϊֹ
	{
		BeginBatchDraw();
		//cout << "(" << pCurrent->pos.row << ", " << pCurrent->pos.col << ") ";//����ֱ�Ӵ�ӡ������ģ����յ㵽��㣻
		result.insert(result.begin(), pCurrent);//ͷ�������������Ȼֻ�������ӡ
		costFinal += pCurrent->pos.g;//�ۼ�·������

		if (!bBeginPoint(pCurrent) && !bEndPoint(pCurrent))//������������յ�
		{
			map0.myMap[pCurrent->pos.row][pCurrent->pos.col] = 6;//��6������յ�·����
			map0.drawRec(pCurrent->pos.row, pCurrent->pos.col, GREEN);//����ɫ������յ�·����
		}

		pCurrent = pCurrent->pParent;//ָ�������ƶ��������
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
	cout << typeName << "�㷨��õ����·���ǣ�" << endl;
	for (auto it = result.begin(); it != result.end(); it++)
	{
		cout << "(" << (*it)->pos.row << ", " << (*it)->pos.col << ")";
		if (!bEndPoint(*it))//�����յ�ʹ�ӡ��ͷ
		{
			cout << " -> ";
		}
	}
	cout << endl;

	cout << "��·�������ǣ�" << costFinal << endl;
}

bool tree::bBeginPoint(treeNode* point)//�Ƿ������
{
	if (point->pos.row == map0.beginP.row && point->pos.col == map0.beginP.col)
	{
		return true;
	}
	return false;
}

bool tree::bEndPoint(treeNode* point)//�Ƿ����յ�
{
	if (point->pos.row == map0.endP.row && point->pos.col == map0.endP.col)
	{
		return true;
	}
	return false;
}

void ShortestPath::shortestPath()//���·��
{
	system("pause");
	tr.updateTree();
}





int main()
{
	initgraph(Swidth, Sheight, EW_SHOWCONSOLE);// easyx��ʼ����ͬʱ����ͼ�δ��ں������	

	ShortestPath sPath;
	sPath.shortestPath();

	system("pause");
	closegraph();
	return 0;
}