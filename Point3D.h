#include <vector>
using namespace std;

template<typename _Tp> class CPoint3D
{
public:
	CPoint3D(void);
	CPoint3D(_Tp _x, _Tp _y, _Tp _z, _Tp _R, _Tp _G, _Tp _B);
	CPoint3D(const CPoint3D& pt);
	~CPoint3D(void);
	//! dot product
	_Tp dot(const CPoint3D& pt) const;
	//! dot product computed in double-precision arithmetics
	double ddot(const CPoint3D& pt) const;
	//! cross product of the 2 3D points
	CPoint3D cross(const CPoint3D& pt) const;
public:
	_Tp x, y, z, R, G, B;
};

typedef CPoint3D<int> CPoint3i;
typedef CPoint3D<float> CPoint3f;
typedef CPoint3D<double> CPoint3d;

template<typename _Tp> inline CPoint3D<_Tp>::CPoint3D() :x(0), y(0), z(0), R(0), G(0), B(0) {}

template<typename _Tp> inline CPoint3D<_Tp>::CPoint3D(_Tp _x, _Tp _y, _Tp _z, _Tp _R, _Tp _G, _Tp _B) { x = _x; y = _y; z = _z; R = _R; G = _G; B = _B; }

template<typename _Tp> inline CPoint3D<_Tp>::CPoint3D(const CPoint3D& pt) : x(pt.x), y(pt.y), z(pt.z), R(pt.R), G(pt.G), B(pt.B) {}

template<typename _Tp> inline CPoint3D<_Tp>::~CPoint3D() {}

template<typename _Tp> inline _Tp CPoint3D<_Tp>::dot(const CPoint3D& pt) const
{
	return CPoint3D<_Tp>(x*pt.x + y * pt.y + z * pt.z);
}

template<typename _Tp> inline double CPoint3D<_Tp>::ddot(const CPoint3D& pt) const
{
	return (double)x*pt.x + (double)y*pt.y + (double)z*pt.z;
}


template<typename _Tp> inline CPoint3D<_Tp> CPoint3D<_Tp>::cross(const CPoint3D<_Tp>& pt) const
{
	return CPoint3D<_Tp>(y*pt.z - z * pt.y, z*pt.x - x * pt.z, x*pt.y - y * pt.x, pt.R, pt.G, pt.B);
}
