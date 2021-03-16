#include <vector>
#include "Point3D.h"
using namespace std;
namespace cv
{
	class COctree
	{
	public:
		struct Node
		{
			Node() {}
			int begin, end;
			float x_min, x_max, y_min, y_max, z_min, z_max;
			bool isLeaf;
			int children[8];
		};

	public:
		COctree(void);
		COctree(const vector<CPoint3f>& points, int minPoints);
		~COctree(void);

		void initTree();
		void buildTree(const vector<CPoint3f>& points, int minPoints);
		const vector<Node>& getNodes() const { return m_nodes; }
		bool ReadFile(char filePath[], vector<CPoint3f>& tempPoint);
		bool WriteFile(char filePath[]);
		void egordicNode(size_t node_Ind,int size);
		void inputArray(vector<CPoint3f> arr, vector<CPoint3f>& tempPoint);
		vector<CPoint3f> outArray();
		vector<CPoint3f> PCDtoArray(char filePath[], vector<CPoint3f>& tempPoint);
		void XYZRGB(float X[], float Y[], float Z[], float R[], float G[], float B[],int size);
	private:
		void buildNext(size_t node_ind);

	private:
		int m_minPoints;
		vector<CPoint3f> m_points;
		vector<Node> m_nodes;
		vector<CPoint3f> m_simplifypoints;
	};
//#define DLLEXPORT __declspec(dllexport)
//#define C_DLL_EXPORT extern "C" __declspec(dllexport)
//	extern "C" {
//		COctree ab_cd;
//		 void XYZRGB(float X[], float Y[], float Z[], float R[], float G[], float B[], int size) {
//			ab_cd.XYZRGB(X,Y,Z,R,G,B,size);
//		}
//	}

}

