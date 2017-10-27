#include "stdafx.h"
#include <stdexcept>
#include "cdt.h"
#include "triangulation.h"

namespace cdt
{
	void Sweep::Triangulate(SweepLine& tcx)
	{
		tcx.InitTriangulation();
		tcx.CreateAdvancingFront(nodes_);
		SweepPoints(tcx);
		FinalizationPolygon(tcx);
	}
	
	void Sweep::SweepPoints(SweepLine& tcx)
	{
		for (size_t i = 1; i < tcx.point_count(); i++)
		{
			Point& point = *tcx.GetPoint(i);
			Node* node = &PointEvent(tcx, point);
			for (unsigned int i = 0; i < point.edge_list.size(); i++)
			{
				EdgeEvent(tcx, point.edge_list[i], node);
			}
		}
	}
	
	void Sweep::FinalizationPolygon(SweepLine& tcx)
	{
		Triangle* t = tcx.front()->head()->next->triangle;
		Point* p = tcx.front()->head()->next->point;
		while (!t->GetConstrainedEdgeCW(*p))
		{
			t = t->NeighborCCW(*p);
		}
		tcx.MeshClean(*t);
	
		Point *p1 = tcx.head_;
		Point *p2 = tcx.tail_;
		for (std::list<Triangle*>::iterator it = tcx.map_.begin(); it != tcx.map_.end();)
		{
			Triangle* t = *it;
	
			if (t->Contains(p1) || t->Contains(p2))
			{
				tcx.RemoveFromMap(*it++);
			}
			else
			{
				it++;
			}
		}
	}
	
	Node& Sweep::PointEvent(SweepLine& tcx, Point& point)
	{
		Node& node = tcx.LocateNode(point);
		Node& new_node = NewFrontTriangle(tcx, point, node);
		
		if (point.xyz.x() <= node.point->xyz.x() + EPSILON)
		{
			Fill(tcx, node);
		}
		
		FillAdvancingFront(tcx, new_node);
		return new_node;
	}
	
	void Sweep::EdgeEvent(SweepLine& tcx, Edge* edge, Node* node)
	{
		tcx.edge_event.constrained_edge = edge;
		tcx.edge_event.right = (edge->p->xyz.x() > edge->q->xyz.x());
		
		if (IsEdgeSideOfTriangle(*node->triangle, *edge->p, *edge->q))
		{
			return;
		}
		
		FillEdgeEvent(tcx, edge, node);
		EdgeEvent(tcx, *edge->p, *edge->q, node->triangle, *edge->q);
	}
	
	void Sweep::EdgeEvent(SweepLine& tcx, Point& ep, Point& eq, Triangle* triangle, Point& point)
	{
		if (IsEdgeSideOfTriangle(*triangle, ep, eq))
		{
			return;
		}
		
		Point* p1 = triangle->PointCCW(point);
		Orientation o1 = Orient2d(eq, *p1, ep);
		if (o1 == COLLINEAR)
		{
			if( triangle->Contains(&eq, p1))
			{
				triangle->MarkConstrainedEdge(&eq, p1 );
				tcx.edge_event.constrained_edge->q = p1;
				triangle = &triangle->NeighborAcross(point);
				EdgeEvent( tcx, ep, *p1, triangle, *p1 );
			}
			else
			{
				std::runtime_error("EdgeEvent - collinear points not supported");
				assert(0);
			}
			return;
		}
		
		Point* p2 = triangle->PointCW(point);
		Orientation o2 = Orient2d(eq, *p2, ep);
		if (o2 == COLLINEAR)
		{
			if( triangle->Contains(&eq, p2))
			{
				triangle->MarkConstrainedEdge(&eq, p2);
				tcx.edge_event.constrained_edge->q = p2;
				triangle = &triangle->NeighborAcross(point);
				EdgeEvent( tcx, ep, *p2, triangle, *p2 );
			}
			else
			{
				std::runtime_error("EdgeEvent - collinear points not supported");
				assert(0);
			}
			return;
		}
		
		if (o1 == o2)
		{
			if (o1 == CW)
			{
				triangle = triangle->NeighborCCW(point);
			}
			else
			{
				triangle = triangle->NeighborCW(point);
			}
			EdgeEvent(tcx, ep, eq, triangle, point);
		}
		else
		{
			FlipEdgeEvent(tcx, ep, eq, triangle, point);
		}
	}
	
	bool Sweep::IsEdgeSideOfTriangle(Triangle& triangle, Point& ep, Point& eq) const
	{
		const int index = triangle.EdgeIndex(&ep, &eq);
		
		if (index != -1)
		{
			triangle.MarkConstrainedEdge(index);
			Triangle* t = triangle.GetNeighbor(index);
			if (t)
			{
				t->MarkConstrainedEdge(&ep, &eq);
			}
			return true;
		}
		return false;
	}
	
	Node& Sweep::NewFrontTriangle(SweepLine& tcx, Point& point, Node& node)
	{
		Triangle* triangle = new Triangle(point, *node.point, *node.next->point);
		
		triangle->MarkNeighbor(*node.triangle);
		tcx.AddToMap(triangle);
		
		Node* new_node = new Node(point);
		nodes_.push_back(new_node);
		
		new_node->next = node.next;
		new_node->prev = &node;
		node.next->prev = new_node;
		node.next = new_node;
		
		if (!Legalize(tcx, *triangle))
		{
			tcx.MapTriangleToNodes(*triangle);
		}
		return *new_node;
	}
	
	void Sweep::Fill(SweepLine& tcx, Node& node)
	{
		Triangle* triangle = new Triangle(*node.prev->point, *node.point, *node.next->point);
		
		triangle->MarkNeighbor(*node.prev->triangle);
		triangle->MarkNeighbor(*node.triangle);
		
		tcx.AddToMap(triangle);

		node.prev->next = node.next;
		node.next->prev = node.prev;
		
		if (!Legalize(tcx, *triangle))
		{
			tcx.MapTriangleToNodes(*triangle);
		}
	}
	
	void Sweep::FillAdvancingFront(SweepLine& tcx, Node& n)
	{
		Node* node = n.next;
		
		while (node->next)
		{
			if (LargeHole_DontFill(node))
				break;
			Fill(tcx, *node);
			node = node->next;
		}

		node = n.prev;
		
		while (node->prev)
		{
			if (LargeHole_DontFill(node)) 
				break;
			Fill(tcx, *node);
			node = node->prev;
		}
		if (n.next && n.next->next)
		{
			const double angle = BasinAngle(n);
			if (angle < PI_3div4)
			{
				FillBasin(tcx, n);
			}
		}
	}
	
	bool Sweep::LargeHole_DontFill(const Node* node) const
	{
		const Node* nextNode = node->next;
		const Node* prevNode = node->prev;
		if (!AngleExceeds90Degrees(node->point, nextNode->point, prevNode->point))
			return false;
		
		const Node* next2Node = nextNode->next;
		if ((next2Node != NULL) && !AngleExceedsPlus90DegreesOrIsNegative(node->point, next2Node->point, prevNode->point))
			return false;
		
		const Node* prev2Node = prevNode->prev;
		
		if ((prev2Node != NULL) && !AngleExceedsPlus90DegreesOrIsNegative(node->point, nextNode->point, prev2Node->point))
			return false;

		return true;
	}

	bool Sweep::AngleExceeds90Degrees(const Point* origin, const Point* pa, const Point* pb) const 
	{
		const double angle = Angle(origin, pa, pb);
		return ((angle > PI_div2) || (angle < -PI_div2));
	}
	
	bool Sweep::AngleExceedsPlus90DegreesOrIsNegative(const Point* origin, const Point* pa, const Point* pb) const
	{
		const double angle = Angle(origin, pa, pb);
		return (angle > PI_div2) || (angle < 0);
	}
	
	double Sweep::Angle(const Point* origin, const Point* pa, const Point* pb) const
	{
		const double px = origin->xyz.x();
		const double py = origin->xyz.y();
		const double ax = pa->xyz.x() - px;
		const double ay = pa->xyz.y() - py;
		const double bx = pb->xyz.x() - px;
		const double by = pb->xyz.y() - py;
		const double x = ax * by - ay * bx;
		const double y = ax * bx + ay * by;
		return atan2(x, y);
	}
	
	double Sweep::BasinAngle(const Node& node) const
	{
		const double ax = node.point->xyz.x() - node.next->next->point->xyz.x();
		const double ay = node.point->xyz.y() - node.next->next->point->xyz.y();
		return atan2(ay, ax);
	}
	
	double Sweep::HoleAngle(const Node& node) const
	{
		const double ax = node.next->point->xyz.x() - node.point->xyz.x();
		const double ay = node.next->point->xyz.y() - node.point->xyz.y();
		const double bx = node.prev->point->xyz.x() - node.point->xyz.x();
		const double by = node.prev->point->xyz.y() - node.point->xyz.y();
		return atan2(ax * by - ay * bx, ax * bx + ay * by);
	}
	
	bool Sweep::Legalize(SweepLine& tcx, Triangle& t)
	{
		for (int i = 0; i < 3; i++)
		{
			if (t.delaunay_edge[i])
				continue;
			
			Triangle* ot = t.GetNeighbor(i);
			
			if (ot)
			{
				Point* p = t.GetPoint(i);
				Point* op = ot->OppositePoint(t, *p);
				int oi = ot->Index(op);
				
				if (ot->constrained_edge[oi] || ot->delaunay_edge[oi])
				{
					t.constrained_edge[i] = ot->constrained_edge[oi];
					continue;
				}
				bool inside = Incircle(*p, *t.PointCCW(*p), *t.PointCW(*p), *op);
				if (inside)
				{
					t.delaunay_edge[i] = true;
					ot->delaunay_edge[oi] = true;
					
					RotateTrianglePair(t, *p, *ot, *op);
					
					
					bool not_legalized = !Legalize(tcx, t);
					if (not_legalized)
					{
						tcx.MapTriangleToNodes(t);
					}
					
					not_legalized = !Legalize(tcx, *ot);
					if (not_legalized)
						tcx.MapTriangleToNodes(*ot);
				
					t.delaunay_edge[i] = false;
					ot->delaunay_edge[oi] = false;
					
					return true;
				}
			}
		}
		return false;
	}
	
	bool Sweep::Incircle(const Point& pa, const Point& pb, const Point& pc, const Point& pd) const
	{
		const double adx = pa.xyz.x() - pd.xyz.x();
		const double ady = pa.xyz.y() - pd.xyz.y();
		const double bdx = pb.xyz.x() - pd.xyz.x();
		const double bdy = pb.xyz.y() - pd.xyz.y();
		
		const double adxbdy = adx * bdy;
		const double bdxady = bdx * ady;
		const double oabd = adxbdy - bdxady;
		
		if (oabd <= 0)
			return false;
		
		const double cdx = pc.xyz.x() - pd.xyz.x();
		const double cdy = pc.xyz.y() - pd.xyz.y();
		
		const double cdxady = cdx * ady;
		const double adxcdy = adx * cdy;
		const double ocad = cdxady - adxcdy;
		
		if (ocad <= 0)
			return false;
		
		const double bdxcdy = bdx * cdy;
		const double cdxbdy = cdx * bdy;
		
		const double alift = adx * adx + ady * ady;
		const double blift = bdx * bdx + bdy * bdy;
		const double clift = cdx * cdx + cdy * cdy;
		
		const double det = alift * (bdxcdy - cdxbdy) + blift * ocad + clift * oabd;
		return det > 0;
	}
	
	void Sweep::RotateTrianglePair(Triangle& t, Point& p, Triangle& ot, Point& op) const
	{
		Triangle* n1, *n2, *n3, *n4;
		n1 = t.NeighborCCW(p);
		n2 = t.NeighborCW(p);
		n3 = ot.NeighborCCW(op);
		n4 = ot.NeighborCW(op);
		
		bool ce1, ce2, ce3, ce4;
		ce1 = t.GetConstrainedEdgeCCW(p);
		ce2 = t.GetConstrainedEdgeCW(p);
		ce3 = ot.GetConstrainedEdgeCCW(op);
		ce4 = ot.GetConstrainedEdgeCW(op);
		
		bool de1, de2, de3, de4;
		de1 = t.GetDelunayEdgeCCW(p);
		de2 = t.GetDelunayEdgeCW(p);
		de3 = ot.GetDelunayEdgeCCW(op);
		de4 = ot.GetDelunayEdgeCW(op);
		
		t.Legalize(p, op);
		ot.Legalize(op, p);
	
		ot.SetDelunayEdgeCCW(p, de1);
		t.SetDelunayEdgeCW(p, de2);
		t.SetDelunayEdgeCCW(op, de3);
		ot.SetDelunayEdgeCW(op, de4);
	
		ot.SetConstrainedEdgeCCW(p, ce1);
		t.SetConstrainedEdgeCW(p, ce2);
		t.SetConstrainedEdgeCCW(op, ce3);
		ot.SetConstrainedEdgeCW(op, ce4);
		
		t.ClearNeighbors();
		ot.ClearNeighbors();
		if (n1) ot.MarkNeighbor(*n1);
		if (n2) t.MarkNeighbor(*n2);
		if (n3) t.MarkNeighbor(*n3);
		if (n4) ot.MarkNeighbor(*n4);
		t.MarkNeighbor(ot);
	}
	
	void Sweep::FillBasin(SweepLine& tcx, Node& node)
	{
		if (Orient2d(*node.point, *node.next->point, *node.next->next->point) == CCW)
		{
			tcx.basin.left_node = node.next->next;
		} 
		else
		{
			tcx.basin.left_node = node.next;
		}
		
		tcx.basin.bottom_node = tcx.basin.left_node;
		while (tcx.basin.bottom_node->next && tcx.basin.bottom_node->point->xyz.y() >= tcx.basin.bottom_node->next->point->xyz.y())
		{
			tcx.basin.bottom_node = tcx.basin.bottom_node->next;
		}
		if (tcx.basin.bottom_node == tcx.basin.left_node)
		{
			return;
		}
		
		tcx.basin.right_node = tcx.basin.bottom_node;
		while (tcx.basin.right_node->next && tcx.basin.right_node->point->xyz.y() < tcx.basin.right_node->next->point->xyz.y()) 
		{
			tcx.basin.right_node = tcx.basin.right_node->next;
		}
		if (tcx.basin.right_node == tcx.basin.bottom_node)
		{
			return;
		}
		
		tcx.basin.width = tcx.basin.right_node->point->xyz.x() - tcx.basin.left_node->point->xyz.x();
		tcx.basin.left_highest = tcx.basin.left_node->point->xyz.y() > tcx.basin.right_node->point->xyz.y();
		
		FillBasinReq(tcx, tcx.basin.bottom_node);
	}
	
	void Sweep::FillBasinReq(SweepLine& tcx, Node* node)
	{
		if (IsShallow(tcx, *node))
		{
			return;
		}
		
		Fill(tcx, *node);
		
		if (node->prev == tcx.basin.left_node && node->next == tcx.basin.right_node)
		{
			return;
		}
		else if (node->prev == tcx.basin.left_node)
		{
			Orientation o = Orient2d(*node->point, *node->next->point, *node->next->next->point);
			if (o == CW)
			{
				return;
			}
			node = node->next;
		}
		else if (node->next == tcx.basin.right_node)
		{
			Orientation o = Orient2d(*node->point, *node->prev->point, *node->prev->prev->point);
			if (o == CCW)
			{
				return;
			}
			node = node->prev;
		}
		else
		{
			if (node->prev->point->xyz.y() < node->next->point->xyz.y())
			{
				node = node->prev;
			}
			else 
			{
				node = node->next;
			}
		}
		
		FillBasinReq(tcx, node);
	}
	
	bool Sweep::IsShallow(SweepLine& tcx, Node& node) const
	{
		double height;
	
		if (tcx.basin.left_highest) 
		{
			height = tcx.basin.left_node->point->xyz.y() - node.point->xyz.y();
		}
		else
		{
			height = tcx.basin.right_node->point->xyz.y() - node.point->xyz.y();
		}
		
		if (tcx.basin.width > height) 
		{
			return true;
		}
		return false;
	}
	
	void Sweep::FillEdgeEvent(SweepLine& tcx, Edge* edge, Node* node)
	{
		if (tcx.edge_event.right)
		{
			FillRightAboveEdgeEvent(tcx, edge, node);
		}
		else
		{
			FillLeftAboveEdgeEvent(tcx, edge, node);
		}
	}
	
	void Sweep::FillRightAboveEdgeEvent(SweepLine& tcx, Edge* edge, Node* node)
	{
		while (node->next->point->xyz.x() < edge->p->xyz.x()) 
		{
			if (Orient2d(*edge->q, *node->next->point, *edge->p) == CCW)
			{
				FillRightBelowEdgeEvent(tcx, edge, *node);
			}
			else
			{
				node = node->next;
			}
		}
	}
	
	void Sweep::FillRightBelowEdgeEvent(SweepLine& tcx, Edge* edge, Node& node)
	{
		if (node.point->xyz.x() < edge->p->xyz.x())
		{
			if (Orient2d(*node.point, *node.next->point, *node.next->next->point) == CCW) 
			{
				FillRightConcaveEdgeEvent(tcx, edge, node);
			}
			else
			{
				FillRightConvexEdgeEvent(tcx, edge, node);
				FillRightBelowEdgeEvent(tcx, edge, node);
			}
		}
	}
	
	void Sweep::FillRightConcaveEdgeEvent(SweepLine& tcx, Edge* edge, Node& node)
	{
		Fill(tcx, *node.next);
		if (node.next->point != edge->p) 
		{
			if (Orient2d(*edge->q, *node.next->point, *edge->p) == CCW)
			{
				if (Orient2d(*node.point, *node.next->point, *node.next->next->point) == CCW)
				{
					FillRightConcaveEdgeEvent(tcx, edge, node);
				}
			}
		}
	}
	
	void Sweep::FillRightConvexEdgeEvent(SweepLine& tcx, Edge* edge, Node& node)
	{
		if (Orient2d(*node.next->point, *node.next->next->point, *node.next->next->next->point) == CCW)
		{
			FillRightConcaveEdgeEvent(tcx, edge, *node.next);
		}
		else
		{
			if (Orient2d(*edge->q, *node.next->next->point, *edge->p) == CCW)
			{
				FillRightConvexEdgeEvent(tcx, edge, *node.next);
			}
		}
	}
	
	void Sweep::FillLeftAboveEdgeEvent(SweepLine& tcx, Edge* edge, Node* node)
	{
		while (node->prev->point->xyz.x() > edge->p->xyz.x())
		{
			if (Orient2d(*edge->q, *node->prev->point, *edge->p) == CW)
			{
				FillLeftBelowEdgeEvent(tcx, edge, *node);
			}
			else
			{
				node = node->prev;
			}
		}
	}
	
	void Sweep::FillLeftBelowEdgeEvent(SweepLine& tcx, Edge* edge, Node& node)
	{
		if (node.point->xyz.x() > edge->p->xyz.x())
		{
			if (Orient2d(*node.point, *node.prev->point, *node.prev->prev->point) == CW)
			{
				FillLeftConcaveEdgeEvent(tcx, edge, node);
			}
			else
			{
				FillLeftConvexEdgeEvent(tcx, edge, node);
				FillLeftBelowEdgeEvent(tcx, edge, node);
			}
		}
	}
	
	void Sweep::FillLeftConvexEdgeEvent(SweepLine& tcx, Edge* edge, Node& node)
	{
		if (Orient2d(*node.prev->point, *node.prev->prev->point, *node.prev->prev->prev->point) == CW)
		{
			FillLeftConcaveEdgeEvent(tcx, edge, *node.prev);
		} 
		else
		{
			if (Orient2d(*edge->q, *node.prev->prev->point, *edge->p) == CW)
			{
				FillLeftConvexEdgeEvent(tcx, edge, *node.prev);
			}
		}
	}
	
	void Sweep::FillLeftConcaveEdgeEvent(SweepLine& tcx, Edge* edge, Node& node)
	{
		Fill(tcx, *node.prev);
		if (node.prev->point != edge->p)
		{
			if (Orient2d(*edge->q, *node.prev->point, *edge->p) == CW)
			{
				if (Orient2d(*node.point, *node.prev->point, *node.prev->prev->point) == CW) 
				{
					FillLeftConcaveEdgeEvent(tcx, edge, node);
				}
			}
		}
	}
	
	void Sweep::FlipEdgeEvent(SweepLine& tcx, Point& ep, Point& eq, Triangle* t, Point& p)
	{
		Triangle& ot = t->NeighborAcross(p);
		Point& op = *ot.OppositePoint(*t, p);
		
		if (InScanArea(p, *t->PointCCW(p), *t->PointCW(p), op)) 
		{
			RotateTrianglePair(*t, p, ot, op);
			tcx.MapTriangleToNodes(*t);
			tcx.MapTriangleToNodes(ot);
			
			if (p == eq && op == ep)
			{
				if (eq == *tcx.edge_event.constrained_edge->q && ep == *tcx.edge_event.constrained_edge->p) 
				{
					t->MarkConstrainedEdge(&ep, &eq);
					ot.MarkConstrainedEdge(&ep, &eq);
					Legalize(tcx, *t);
					Legalize(tcx, ot);
				}
			}
			else
			{
				Orientation o = Orient2d(eq, op, ep);
				t = &NextFlipTriangle(tcx, (int)o, *t, ot, p, op);
				FlipEdgeEvent(tcx, ep, eq, t, p);
			}
		}
		else
		{
			Point& newP = NextFlipPoint(ep, eq, ot, op);
			FlipScanEdgeEvent(tcx, ep, eq, *t, ot, newP);
			EdgeEvent(tcx, ep, eq, t, p);
		}
	}
	
	Triangle& Sweep::NextFlipTriangle(SweepLine& tcx, int o, Triangle& t, Triangle& ot, Point& p, Point& op)
	{
		if (o == CCW) 
		{
			int edge_index = ot.EdgeIndex(&p, &op);
			ot.delaunay_edge[edge_index] = true;
			Legalize(tcx, ot);
			ot.ClearDelunayEdges();
			return t;
		}
		int edge_index = t.EdgeIndex(&p, &op);
		
		t.delaunay_edge[edge_index] = true;
		Legalize(tcx, t);
		t.ClearDelunayEdges();
		return ot;
	}
	
	Point& Sweep::NextFlipPoint(Point& ep, Point& eq, Triangle& ot, Point& op) const
	{
		Orientation o2d = Orient2d(eq, op, ep);
		if (o2d == CW)
		{
			return *ot.PointCCW(op);
		}
		else if (o2d == CCW) 
		{
			return *ot.PointCW(op);
		}
		throw std::runtime_error("[Unsupported] Opposing point on constrained edge");
	}
	
	void Sweep::FlipScanEdgeEvent(SweepLine& tcx, Point& ep, Point& eq, Triangle& flip_triangle, Triangle& t, Point& p)
	{
		Triangle& ot = t.NeighborAcross(p);
		Point& op = *ot.OppositePoint(t, p);

		if (InScanArea(eq, *flip_triangle.PointCCW(eq), *flip_triangle.PointCW(eq), op))
		{
			FlipEdgeEvent(tcx, eq, op, &ot, op);
		}
		else
		{
			Point& newP = NextFlipPoint(ep, eq, ot, op);
			FlipScanEdgeEvent(tcx, ep, eq, flip_triangle, ot, newP);
		}
	}
	
	Sweep::~Sweep()
	{
		for(size_t i = 0; i < nodes_.size(); i++) 
		{
			delete nodes_[i];
		}
	}
}

