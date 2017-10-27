#include "stdafx.h"
#include "shapes.h"
#include <iostream>

namespace cdt
{
	
	Triangle::Triangle(Point& a, Point& b, Point& c)
	{
		points_[0] = &a; points_[1] = &b; points_[2] = &c;
		neighbors_[0] = NULL; neighbors_[1] = NULL; neighbors_[2] = NULL;
		constrained_edge[0] = constrained_edge[1] = constrained_edge[2] = false;
		delaunay_edge[0] = delaunay_edge[1] = delaunay_edge[2] = false;
		interior_ = false;
	}
	
	Triangle::~Triangle()
	{
	}
	void Triangle::MarkNeighbor(Point* p1, Point* p2, Triangle* t)
	{
		if ((p1 == points_[2] && p2 == points_[1]) || (p1 == points_[1] && p2 == points_[2]))
			neighbors_[0] = t;
		else if ((p1 == points_[0] && p2 == points_[2]) || (p1 == points_[2] && p2 == points_[0]))
			neighbors_[1] = t;
		else if ((p1 == points_[0] && p2 == points_[1]) || (p1 == points_[1] && p2 == points_[0]))
			neighbors_[2] = t;
		else
			assert(0);
	}
	void Triangle::MarkNeighbor(Triangle& t)
	{
		if (t.Contains(points_[1], points_[2]))
		{
			neighbors_[0] = &t;
			t.MarkNeighbor(points_[1], points_[2], this);
		} 
		
		else if (t.Contains(points_[0], points_[2]))
		{
			neighbors_[1] = &t;
			t.MarkNeighbor(points_[0], points_[2], this);
		}
		else if (t.Contains(points_[0], points_[1]))
		{
			neighbors_[2] = &t;
			t.MarkNeighbor(points_[0], points_[1], this);
		}
	}
	
	void Triangle::Clear()
	{
		Triangle *t;
		for( int i=0; i<3; i++ )
		{
			t = neighbors_[i];
			if( t != NULL )
			{
				t->ClearNeighbor( this );
			}
		}
		ClearNeighbors();
		points_[0]=points_[1]=points_[2] = NULL;
	}
	
	void Triangle::ClearNeighbor(const Triangle *triangle )
	{
		if( neighbors_[0] == triangle )
		{
			neighbors_[0] = NULL;
		}
		else if( neighbors_[1] == triangle )
		{
			neighbors_[1] = NULL;
		}
		else
		{
			neighbors_[2] = NULL;
		}
	}
	
	void Triangle::ClearNeighbors()
	{
		neighbors_[0] = NULL;
		neighbors_[1] = NULL;
		neighbors_[2] = NULL;
	}
	
	void Triangle::ClearDelunayEdges()
	{
		delaunay_edge[0] = delaunay_edge[1] = delaunay_edge[2] = false;
	}
	
	Point* Triangle::OppositePoint(Triangle& t, const Point& p) const
	{
		Point *cw = t.PointCW(p);
		return PointCW(*cw);
	}
	
	void Triangle::Legalize(Point& point)
	{
		points_[1] = points_[0];
		points_[0] = points_[2];
		points_[2] = &point;
	}
	
	void Triangle::Legalize(Point& opoint, Point& npoint)
	{
		if (&opoint == points_[0])
		{
			points_[1] = points_[0];
			points_[0] = points_[2];
			points_[2] = &npoint;
		}
		else if (&opoint == points_[1])
		{
			points_[2] = points_[1];
			points_[1] = points_[0];
			points_[0] = &npoint;
		} 
		else if (&opoint == points_[2])
		{
			points_[0] = points_[2];
			points_[2] = points_[1];
			points_[1] = &npoint;
		}
		else
		{
			assert(0);
		}
	}
	
	int Triangle::Index(const Point* p) const
	{
		if (p == points_[0])
		{
			return 0;
		}
		else if (p == points_[1])
		{
			return 1;
		}
		else if (p == points_[2])
		{
			return 2;
		}
		assert(0);
		return -1;
	}
	
	int Triangle::EdgeIndex(const Point* p1, const Point* p2) const
	{
		if (points_[0] == p1) 
		{
			if (points_[1] == p2) 
			{
				return 2;
			}
			else if (points_[2] == p2)
			{
				return 1;
			}
		}
		else if (points_[1] == p1)
		{
			if (points_[2] == p2)
			{
				return 0;
			}
			else if (points_[0] == p2)
			{
				return 2;
			}
		}
		else if (points_[2] == p1)
		{
			if (points_[0] == p2)
			{
				return 1;
			}
			else if (points_[1] == p2)
			{
				return 0;
			}
		}
		return -1;
	}
	
	void Triangle::MarkConstrainedEdge(int index)
	{
		constrained_edge[index] = true;
	}
	
	void Triangle::MarkConstrainedEdge(Edge& edge)
	{
		MarkConstrainedEdge(edge.p, edge.q);
	}
	
	void Triangle::MarkConstrainedEdge(Point* p, Point* q)
	{
		if ((q == points_[0] && p == points_[1]) || (q == points_[1] && p == points_[0]))
		{
			constrained_edge[2] = true;
		}
		else if ((q == points_[0] && p == points_[2]) || (q == points_[2] && p == points_[0]))
		{
			constrained_edge[1] = true;
		}
		else if ((q == points_[1] && p == points_[2]) || (q == points_[2] && p == points_[1]))
		{
			constrained_edge[0] = true;
		}
	}
	
	Point* Triangle::PointCW(const Point& point) const
	{
		if (&point == points_[0]) 
		{
			return points_[2];
		}
		else if (&point == points_[1])
		{
			return points_[0];
		}
		else if (&point == points_[2]) 
		{
			return points_[1];
		}
		assert(0);
		return NULL;
	}
	
	Point* Triangle::PointCCW(const Point& point) const
	{
		if (&point == points_[0])
		{
			return points_[1];
		}
		else if (&point == points_[1])
		{
			return points_[2];
		} 
		else if (&point == points_[2])
		{
			return points_[0];
		}
		assert(0);
		return NULL;
	}
	
	Triangle* Triangle::NeighborCW(const Point& point) const
	{
		if (&point == points_[0])
		{
			return neighbors_[1];
		}
		else if (&point == points_[1])
		{
			return neighbors_[2];
		}
		return neighbors_[0];
	}
	
	Triangle* Triangle::NeighborCCW(const Point& point) const
	{
		if (&point == points_[0])
		{
			return neighbors_[2];
		}
		else if (&point == points_[1])
		{
			return neighbors_[0];
		}
		return neighbors_[1];
	}
	
	bool Triangle::GetConstrainedEdgeCCW(const Point& p) const
	{
		if (&p == points_[0])
		{
			return constrained_edge[2];
		}
		else if (&p == points_[1])
		{
			return constrained_edge[0];
		}
		return constrained_edge[1];
	}
	
	bool Triangle::GetConstrainedEdgeCW(const Point& p) const
	{
		if (&p == points_[0])
		{
			return constrained_edge[1];
		} 
		else if (&p == points_[1])
		{
			return constrained_edge[2];
		}
		return constrained_edge[0];
	}
	
	void Triangle::SetConstrainedEdgeCCW(const Point& p, bool ce)
	{
		if (&p == points_[0])
		{
			constrained_edge[2] = ce;
		}
		else if (&p == points_[1])
		{
			constrained_edge[0] = ce;
		}
		else
		{
			constrained_edge[1] = ce;
		}
	}
	
	void Triangle::SetConstrainedEdgeCW(const Point& p, bool ce)
	{
		if (&p == points_[0])
		{
			constrained_edge[1] = ce;
		}
		else if (&p == points_[1])
		{
			constrained_edge[2] = ce;
		}
		else
		{
			constrained_edge[0] = ce;
		}
	}
	
	bool Triangle::GetDelunayEdgeCCW(const Point& p) const
	{
		if (&p == points_[0])
		{
			return delaunay_edge[2];
		}
		else if (&p == points_[1])
		{
			return delaunay_edge[0];
		}
		return delaunay_edge[1];
	}
	
	bool Triangle::GetDelunayEdgeCW(const Point& p) const
	{
		if (&p == points_[0])
		{
			return delaunay_edge[1];
		}
		else if (&p == points_[1])
		{
			return delaunay_edge[2];
		}
		return delaunay_edge[0];
	}
	
	void Triangle::SetDelunayEdgeCCW(const Point& p, bool e)
	{
		if (&p == points_[0])
		{
			delaunay_edge[2] = e;
		}
		else if (&p == points_[1])
		{
			delaunay_edge[0] = e;
		}
		else
		{
			delaunay_edge[1] = e;
		}
	}
	
	void Triangle::SetDelunayEdgeCW(const Point& p, bool e)
	{
		if (&p == points_[0])
		{
			delaunay_edge[1] = e;
		}
		else if (&p == points_[1])
		{
			delaunay_edge[2] = e;
		}
		else
		{
			delaunay_edge[0] = e;
		}
	}
	
	Triangle& Triangle::NeighborAcross(const Point& opoint) const
	{
		if (&opoint == points_[0])
		{
			return *neighbors_[0];
		}
		else if (&opoint == points_[1])
		{
			return *neighbors_[1];
		}
		return *neighbors_[2];
	}
	
	void Triangle::DebugPrint() const
	{
		using namespace std;
		cout << points_[0]->xyz.x() << "," << points_[0]->xyz.y() <<"," << points_[0]->xyz.z() << " ";
		cout << points_[1]->xyz.x() << "," << points_[1]->xyz.y() << "," << points_[1]->xyz.z() << " ";
		cout << points_[2]->xyz.x() << "," << points_[2]->xyz.y() << "," << points_[2]->xyz.z() << endl;
	}
}