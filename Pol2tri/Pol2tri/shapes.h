#ifndef SHAPES_H
#define SHAPES_H

#include <vector>
#include <cstddef>
#include <assert.h>
#include <cmath>
#include <osg/Array>
#include "export.h"

namespace cdt
{
	struct Edge;
	
	struct POLY2TRI_DLL Point
	{
		int index, type;
		osg::Vec3d xyz;
		Point()
		{
			index = 0;
			type = 0;
			xyz.x() = 0;
			xyz.y() = 0;
			xyz.z() = 0;
		}

		std::vector<Edge*> edge_list;
		
		Point(osg::Vec3d p) : xyz(p){};
		Point(int type, int index, osg::Vec3d xyz) :type(type), index(index), xyz(xyz){};
		
		Point(double x, double y)
		{
			xyz.x() = x;
			xyz.y() = y;
			xyz.z() = 0.0;
		}

		Point(double x, double y, double z)
		{
			xyz.x() = x;
			xyz.y() = y;
			xyz.z() = z;
		}

		void set_zero()
		{
			xyz.x() = 0.0;
			xyz.y() = 0.0;
			xyz.z() = 0.0;
		}
		
		void set(double x_, double y_,double z_)
		{
			xyz.x() = x_;
			xyz.y() = y_;
			xyz.z() = z_;
		}
		
		Point operator -() const
		{
			Point v;
			v.set(-xyz.x(), -xyz.y(),-xyz.z());
			return v;
		}
		
		void operator +=(const Point& v)
		{
			xyz.x() += v.xyz.x();
			xyz.y() += v.xyz.y();
			xyz.z() += v.xyz.z();
		}
		
		void operator -=(const Point& v)
		{
			xyz.x() -= v.xyz.x();
			xyz.y() -= v.xyz.y();
			xyz.z() -= v.xyz.z();
		}
		
		void operator *=(double a)
		{
			xyz.x() *= a;
			xyz.y() *= a;
			xyz.z() *= a;
		}

		void operator /=(double a)
		{
			if (fabs(a) > 0.0000001)
			xyz.x() /= a;
			xyz.y() /= a;
			xyz.z() /= a;
		}
		
		double Length() const
		{
			return xyz.length();
		}
		
		double Normalize()
		{
			return xyz.normalize();
		}
	};
	
	struct POLY2TRI_DLL Edge
	{
		Point* p, *q; 
		
		Edge(Point& p1, Point& p2) : p(&p1), q(&p2)
		{
			if (p1.xyz.y() > p2.xyz.y())
			{
				q = &p1;
				p = &p2;
			} 
			else if (fabs(p1.xyz.y() - p2.xyz.y()) < 0.0000001)
			{
				if (p1.xyz.x() > p2.xyz.x())
				{
					q = &p1;
					p = &p2;
				} 
				else if (fabs(p1.xyz.x() - p2.xyz.x()) < 0.0000001) 
				{
					assert(false);
				}
			}
			
			q->edge_list.push_back(this);
		}
	};
	
	class POLY2TRI_DLL Triangle
	{
	public:
		Triangle(Point& a, Point& b, Point& c);
		
		bool constrained_edge[3];
		bool delaunay_edge[3];
		
		Point* GetPoint(int index)const; 
		Point* PointCW(const Point& point)const; 
		Point* PointCCW(const Point& point)const;
		Point* OppositePoint(Triangle& t, const Point& p)const;
		
		Triangle* GetNeighbor(int index)const;
		void MarkNeighbor(Point* p1, Point* p2, Triangle* t); 
		void MarkNeighbor(Triangle& t);
		
		void MarkConstrainedEdge(int index);
		void MarkConstrainedEdge(Edge& edge);
		void MarkConstrainedEdge(Point* p, Point* q);
		
		int Index(const Point* p)const;  
		int EdgeIndex(const Point* p1, const Point* p2)const; 
		
		Triangle* NeighborCW(const Point& point)const; 
		Triangle* NeighborCCW(const Point& point)const; 
		bool GetConstrainedEdgeCCW(const Point& p)const; 
		bool GetConstrainedEdgeCW(const Point& p)const; 
		void SetConstrainedEdgeCCW(const Point& p, bool ce); 
		void SetConstrainedEdgeCW(const Point& p, bool ce);
		bool GetDelunayEdgeCCW(const Point& p)const;
		bool GetDelunayEdgeCW(const Point& p)const;
		void SetDelunayEdgeCCW(const Point& p, bool e); 
		void SetDelunayEdgeCW(const Point& p, bool e);
		
		bool Contains(const Point* p)const;
		bool Contains(const Edge& e)const; 
		bool Contains(const Point* p, const Point* q)const;
		void Legalize(Point& point);
		void Legalize(Point& opoint, Point& npoint); 
		
		void Clear();
		void ClearNeighbor(const Triangle *triangle);
		void ClearNeighbors();
		void ClearDelunayEdges();
		
		inline bool IsInterior()const;
		inline void IsInterior(bool b);
		Triangle& NeighborAcross(const Point& opoint)const;
		
		void DebugPrint()const;
		~Triangle();

	private:
		Point* points_[3];
		Triangle* neighbors_[3];
		bool interior_;
	};
	
	POLY2TRI_DLL inline bool cmp(const Point* a, const Point* b) 
	{
		if (a->xyz.y() < b->xyz.y()) 
		{
			return true;
		} 
		else if ((a->xyz.y() == b->xyz.y()) )
		{
			if (a->xyz.x() < b->xyz.x()) 
			{
				return true;
			}
		}
		return false;
	}
	
	POLY2TRI_DLL inline Point operator +(const Point& a, const Point& b)
	{
		return Point(a.xyz.x() + b.xyz.x(), a.xyz.y() + b.xyz.y(), a.xyz.z() + a.xyz.z());
	}
	
	POLY2TRI_DLL inline Point operator -(const Point& a, const Point& b)
	{
		return Point(a.xyz.x() - b.xyz.x(), a.xyz.y() - b.xyz.y(), a.xyz.z() - b.xyz.z());
	}
	
	POLY2TRI_DLL inline Point operator *(double s, const Point& a)
	{
		return Point(s * a.xyz.x(), s * a.xyz.y(), s*a.xyz.z());
	}
	
	POLY2TRI_DLL inline bool operator ==(const Point& a, const Point& b)
	{
		if (fabs(a.xyz.x() - b.xyz.x()) < 0.0000001 && fabs(a.xyz.y()-b.xyz.y()) < 0.0000001)
			return true;
		return false;
	}
	
	POLY2TRI_DLL inline bool operator !=(const Point& a, const Point& b)
	{
		if (fabs(a.xyz.x() - b.xyz.x()) >= 0.0000001 && fabs(a.xyz.y() - b.xyz.y()) >= 0.0000001)
			return true;
		return false;
	}
	
	POLY2TRI_DLL inline double Dot(const Point& a, const Point& b)
	{
		return a.xyz.x() * b.xyz.x() + a.xyz.y() * b.xyz.y();
	}
	
	POLY2TRI_DLL inline double Cross(const Point& a, const Point& b)
	{
		return a.xyz.x() * b.xyz.y() - a.xyz.y() * b.xyz.x();
	}
	
	POLY2TRI_DLL inline Point Cross(const Point& a, double s)
	{
		return Point(s * a.xyz.y(), -s * a.xyz.x());
	}
	
	POLY2TRI_DLL inline Point Cross(double s, const Point& a)
	{
		return Point(-s * a.xyz.y(), s * a.xyz.x());
	}
	
	POLY2TRI_DLL inline Point* Triangle::GetPoint(int index)const
	{
		return points_[index];
	}
	
	POLY2TRI_DLL inline Triangle* Triangle::GetNeighbor(int index)const
	{
		return neighbors_[index];
	}
	
	POLY2TRI_DLL inline bool Triangle::Contains(const Point* p)const
	{
		return p == points_[0] || p == points_[1] || p == points_[2];
	}

	POLY2TRI_DLL inline bool Triangle::Contains(const Edge& e)const
	{
	  return Contains(e.p) && Contains(e.q);
	}

	inline bool Triangle::Contains(const Point* p, const Point* q)const
	{
	  return Contains(p) && Contains(q);
	}

	inline bool Triangle::IsInterior()const
	{
	  return interior_;
	}

	inline void Triangle::IsInterior(bool b)
	{
	  interior_ = b;
	}
}

#endif