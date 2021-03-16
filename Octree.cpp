#include <stdio.h>
#include "Octree.h"
#include <iostream>
#include <fstream>
#include <string.h>  
#include <string> 
#include <math.h>
using std::string;


namespace
{
	using namespace cv;
	const size_t MAX_LEAFS = 8;
	
	void FillMinMax(const vector<CPoint3f> &points, COctree::Node& _Node)
	{
		_Node.x_max = _Node.y_max = _Node.z_max = -99999;
		_Node.x_min = _Node.y_min = _Node.z_min  = 99999;
		for (size_t i = 0; i < points.size(); i++)
		{
			const CPoint3f& point = points[i];

			if (point.x > _Node.x_max)
				_Node.x_max = point.x;

			if (point.y > _Node.y_max)
				_Node.y_max = point.y;

			if (point.z > _Node.z_max)
				_Node.z_max = point.z;

			if (point.x < _Node.x_min)
				_Node.x_min = point.x;

			if (point.y < _Node.y_min)
				_Node.y_min = point.y;

			if (point.z < _Node.z_min)
				_Node.z_min = point.z;
		}
	}

	size_t findSubboxForPoint(const CPoint3f& point, const COctree::Node& _Node)
	{
		size_t ind_x = (point.x < (_Node.x_max + _Node.x_min) / 2) ? 0 : 1;

		size_t ind_y = (point.y < (_Node.y_max + _Node.y_min) / 2) ? 0 : 1;

		size_t ind_z = (point.z < (_Node.z_max + _Node.z_min) / 2) ? 0 : 1;

		return (ind_x << 2) + (ind_y << 1) + (ind_z << 0);
	}

	void initChildBox(const COctree::Node& parent, size_t boxIndex, COctree::Node& child)
	{
		child.x_max = child.x_min = (parent.x_max + parent.x_min) / 2;
		child.y_max = child.y_min = (parent.y_max + parent.y_min) / 2;
		child.z_max = child.z_min = (parent.z_max + parent.z_min) / 2;

		if ((boxIndex >> 0) & 1)
			child.z_max = parent.z_max;
		else
			child.z_min = parent.z_min;

		if ((boxIndex >> 1) & 1)
			child.y_max = parent.y_max;
		else
			child.y_min = parent.y_min;

		if ((boxIndex >> 2) & 1)
			child.x_max = parent.x_max;
		else
			child.x_min = parent.x_min;
	}

	double GetDistance(CPoint3f point1, CPoint3f point2)
	{
		double x2 = (point1.x - point2.x) * (point1.x - point2.x);
		double y2 = (point1.y - point2.y) * (point1.y - point2.y);
		double z2 = (point1.z - point2.z) * (point1.z - point2.z);
		return sqrt(x2 + y2 + z2);
	}
}

namespace cv
{
	COctree::COctree(void)
	{

	}

	COctree::COctree(const vector<CPoint3f>& points, int minPoints)
	{
		buildTree(points, minPoints);
	}

	COctree::~COctree(void)
	{
	}

	void COctree::buildTree(const vector<CPoint3f>& points, int minPoints)
	{
		m_points.resize(points.size());
		std::copy(points.begin(), points.end(), m_points.begin());

		m_minPoints = minPoints;
		m_nodes.clear();
		m_nodes.push_back(Node());
		Node& root = m_nodes[0];
		FillMinMax(m_points, root);

		root.isLeaf = true;
		root.begin = 0;
		root.end = (int)m_points.size();

		for (size_t i = 0; i < MAX_LEAFS; i++)
			root.children[i] = 0;

		if ((root.end - root.begin) > m_minPoints)
		{
			root.isLeaf = false;
			buildNext(0);
		}
	}

	void COctree::buildNext(size_t nodeInd)
	{
		size_t size = m_nodes[nodeInd].end - m_nodes[nodeInd].begin;

		vector<size_t> boxBorders(MAX_LEAFS + 1, 0);
		vector<size_t> boxIndices(size);
		vector<CPoint3f> tempPoints(size);

		for (int i = m_nodes[nodeInd].begin, j = 0; i < m_nodes[nodeInd].end; ++i, ++j)
		{
			const CPoint3f& p = m_points[i];

			size_t subboxInd = findSubboxForPoint(p, m_nodes[nodeInd]);

			boxBorders[subboxInd + 1]++;
			boxIndices[j] = subboxInd;
			tempPoints[j] = p;
		}

		for (size_t i = 1; i < boxBorders.size(); ++i)
			boxBorders[i] += boxBorders[i - 1]; //盒子第一个点的序号，box的begin

		vector<size_t> writeInds(boxBorders.begin(), boxBorders.end());

		for (size_t i = 0; i < size; ++i)
		{
			size_t boxIndex = boxIndices[i];
			CPoint3f& curPoint = tempPoints[i];

			size_t copyTo = m_nodes[nodeInd].begin + writeInds[boxIndex]++;
			m_points[copyTo] = curPoint; //排序，将无序点云按所在盒子序号排列
		}

		for (size_t i = 0; i < MAX_LEAFS; ++i)
		{
			if (boxBorders[i] == boxBorders[i + 1])//说明第i个盒子没点
				continue;

			m_nodes.push_back(Node());
			Node& child = m_nodes.back();
			initChildBox(m_nodes[nodeInd], i, child);

			child.isLeaf = true;
			child.begin = m_nodes[nodeInd].begin + (int)boxBorders[i + 0];
			child.end = m_nodes[nodeInd].begin + (int)boxBorders[i + 1];
			for (size_t k = 0; k < MAX_LEAFS; k++)
				child.children[k] = 0;

			m_nodes[nodeInd].children[i] = (int)(m_nodes.size() - 1);
			if ((child.end - child.begin) > m_minPoints)
			{
				child.isLeaf = false;
				buildNext(m_nodes.size() - 1);
			}
		}
	}

	bool COctree::ReadFile(char filePath[], vector<CPoint3f>& tempPoint)
	{
		int TotalPointNum = 0;
//		errno_t err;
		FILE* file;

		file = fopen(filePath, "r");
		/*err = fopen_s(&file, filePath,"r");
		if (err != 0) { */
		if (file == NULL) {
			cout << filePath << endl;
			cout << "can not open file!" << endl;
			return false;
		}

		char str[30];
		string tempstr = "ascii";
		while (strcmp(str, tempstr.c_str()))
		{
			fscanf(file, "%s", str);
			if (strcmp(str, "POINTS") == 0)
				fscanf(file, " %d", &TotalPointNum);
		}
		cout << TotalPointNum << endl;
		for (int i = 0; i < TotalPointNum; i++)
		{
			CPoint3f NewPoint;
			fscanf(file, "%f %f %f %f %f %f", &NewPoint.x, &NewPoint.y, &NewPoint.z,&NewPoint.R,&NewPoint.G,&NewPoint.B);
			tempPoint.push_back(NewPoint);
		}
		return true;
	}

	void COctree::inputArray(vector<CPoint3f> arr, vector<CPoint3f>& tempPoint) {
		int row = arr.size();
		for (int i = 0; i < row; i++)
		{
			tempPoint.push_back(arr[i]);
		}
	}


	void COctree::egordicNode(size_t node_Ind, int size)
	{
		if (m_nodes.empty() == true)
			return;

		Node& cur = m_nodes[node_Ind];

		if (!cur.isLeaf)
		{
			if (cur.children[0] != 0)
				egordicNode(cur.children[0],size);

			if (cur.children[1] != 0)
				egordicNode(cur.children[1], size);

			if (cur.children[2] != 0)
				egordicNode(cur.children[2], size);

			if (cur.children[3] != 0)
				egordicNode(cur.children[3], size);

			if (cur.children[4] != 0)
				egordicNode(cur.children[4], size);

			if (cur.children[5] != 0)
				egordicNode(cur.children[5], size);

			if (cur.children[6] != 0)
				egordicNode(cur.children[6], size);

			if (cur.children[7] != 0)
				egordicNode(cur.children[7], size);
		}
		else
		{
			if ((cur.end - cur.begin) > size)
			{
				cout << "Node" << node_Ind << ":" << endl;
				CPoint3f Center;
				CPoint3f NearPoint;
				Center.x = (cur.x_max + cur.x_min) / 2;
				Center.y = (cur.y_max + cur.y_min) / 2;
				Center.z = (cur.z_max + cur.z_min) / 2;

				double NearDistance = 99999;
				for (int i = cur.begin; i < cur.end; i++)
				{
					const CPoint3f point = m_points[i];

					double Distance = GetDistance(Center, point);

					if (Distance < NearDistance)
					{
						NearPoint = point;
						NearDistance = Distance;
					}
				}
				m_simplifypoints.push_back(NearPoint);
			}
		}
	}
	bool COctree::WriteFile(char filePath[]) // 将压缩后的文件以.pcd格式储存
	{
		if (m_simplifypoints.empty() == true)
			return false;

		ofstream fOutput(filePath, ios::out);

		if (fOutput)
		{
			fOutput << "# .PCD v0.7 - Point Cloud Data file format\n";
			fOutput << "VERSION 0.7\n";
			fOutput << "FIELDS x y z R G B\n";
			fOutput << "SIZE 4 4 4 4 4 4\n";
			fOutput << "TYPE F F F F F F\n";
			fOutput << "COUNT 1 1 1 1 1 1\n";
			fOutput << "WIDTH " << m_simplifypoints.size() << '\n';
			fOutput << "HEIGHT 1\n";
			fOutput << "VIEWPOINT 0 0 0 1 0 0 0\n";
			fOutput << "POINTS " << m_simplifypoints.size() << '\n';
			fOutput << "DATA ascii\n";

			for (int i = 0; i < m_simplifypoints.size(); i++)
			{
				fOutput << m_simplifypoints[i].x << ' ' << m_simplifypoints[i].y << ' ' << m_simplifypoints[i].z << ' ' << m_simplifypoints[i].R << ' ' << m_simplifypoints[i].G << ' ' << m_simplifypoints[i].B << '\n';
			}
		}
		else
			return false;
		cout << filePath << endl;

		return true;
	}

	vector<CPoint3f> COctree::outArray() {		
		if (m_simplifypoints.empty() == true)cout << "cloud point is empty!" << endl;		
		return m_simplifypoints;
	}


	vector<CPoint3f> COctree::PCDtoArray(char filePath[], vector<CPoint3f>& tempPoint) {
	int TotalPointNum = 0;
//	errno_t err;
	FILE* file;
	file = fopen(filePath, "r");
	/*err = fopen_s(&file, filePath,"r");
	if (err != 0) { */
	if (file == NULL) {
		cout << filePath << endl;
	cout << "can not open file!" << endl;
	}

		char str[30];
		string tempstr = "ascii";
		while (strcmp(str, tempstr.c_str()))
		{
			fscanf(file, "%s", str);
			if (strcmp(str, "POINTS") == 0)
				fscanf(file, " %d", &TotalPointNum);
		}
		cout << TotalPointNum << endl;
		for (int i = 0; i < TotalPointNum; i++)
		{
			CPoint3f NewPoint1;
			vector<float> a;
			fscanf(file, "%f %f %f", &NewPoint1.x, &NewPoint1.y, &NewPoint1.z);
			tempPoint.push_back(NewPoint1);
		
		}
		return tempPoint;
	}
	void COctree::initTree()
	{
		m_points.clear();
		m_simplifypoints.clear();
		m_simplifypoints.clear();
	}
	void  COctree::XYZRGB(float X[], float Y[], float Z[], float R[], float G[], float B[],int size) {
			vector<CPoint3f> points;
			char fileout[] = "E:\\q2.pcd";
		for (int i = 0; i < 1000; i++)
		{
			CPoint3f newpoint(X[i], Y[i], Z[i], R[i], G[i], B[i]);
			points.push_back(newpoint);
		}
		initTree();
		buildTree(points, 10);
		egordicNode(0, 1);
		WriteFile(fileout);
	}
}
extern "C" {
	COctree ab_cd;
	 void XYZRGB(float X[], float Y[], float Z[], float R[], float G[], float B[], int size) {
		ab_cd.XYZRGB(X, Y, Z, R, G, B, size);
	}
}

int main(int argc, char* argv[])
{
	COctree octree;
	float X[1000], Y[1000], Z[1000], R[1000], G[1000], B[1000];
	for (int i = 0; i < 1000; i++) {
		X[i] = 0.5f * i;
		Y[i] = 0.5f * i+1;
		Z[i] = 0.5f * i+2;
		R[i] = 0.5f * i+3;
		G[i] = 0.5f * i+4;
		B[i] = 0.5f * i+5;
	}
	octree.XYZRGB(X,Y,Z,R,G,B,1000);
	return 0;
}

