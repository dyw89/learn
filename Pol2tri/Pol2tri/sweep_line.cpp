#include "stdafx.h"
#include <algorithm>
#include "cdt.h"

namespace cdt
{
	SweepLine::SweepLine(const std::vector<Point*>& polyline) : points_(polyline),
		front_(0),
		head_(0),
		tail_(0),
		af_head_(0),
		af_middle_(0),
		af_tail_(0)
	{
		InitEdges(points_);
	}
	
	void SweepLine::AddHole(const std::vector<Point*>& polyline)
	{
		InitEdges(polyline);
		for(unsigned int i = 0; i < polyline.size(); i++)
		{
			points_.push_back(polyline[i]);
		}
	}
	
	void SweepLine::AddPoint(Point* point) 
	{
		points_.push_back(point);
	}
	
	std::vector<Triangle*> &SweepLine::GetTriangles()
	{
		return triangles_;
	}
	
	std::list<Triangle*> &SweepLine::GetMap()
	{
		return map_;
	}
	
	void SweepLine::InitTriangulation()
	{
		double xmax(points_[0]->xyz.x()), xmin(points_[0]->xyz.x());
		double ymax(points_[0]->xyz.y()), ymin(points_[0]->xyz.y());
		
		for (unsigned int i = 0; i < points_.size(); i++)
		{
			Point& p = *points_[i];
			if (p.xyz.x() > xmax)
				xmax = p.xyz.x();
			if (p.xyz.x() < xmin)
				xmin = p.xyz.x();
			if (p.xyz.y() > ymax)
				ymax = p.xyz.y();
			if (p.xyz.y() < ymin)
				ymin = p.xyz.y();
		}
		
		double dx = kAlpha * (xmax - xmin);
		double dy = kAlpha * (ymax - ymin);
		head_ = new Point(xmax + dx, ymin - dy);
		tail_ = new Point(xmin - dx, ymin - dy);
		
		// Sort points along y-axis yÖáÉýÐòÅÅÐò
		std::sort(points_.begin(), points_.end(), cmp);
	}
	
	void SweepLine::InitEdges(const std::vector<Point*>& polyline)
	{
		size_t num_points = polyline.size();
		for (size_t i = 0; i < num_points; i++)
		{
			size_t j = i < num_points - 1 ? i + 1 : 0;
			edge_list.push_back(new Edge(*polyline[i], *polyline[j]));
		}
	}
	
	Point* SweepLine::GetPoint(size_t index) const
	{
		return points_[index];
	}
	
	void SweepLine::AddToMap(Triangle* triangle)
	{
		map_.push_back(triangle);
	}
	
	Node& SweepLine::LocateNode(const Point& point)
	{
		return *front_->LocateNode(point.xyz.x());
	}
	
	void SweepLine::CreateAdvancingFront(const std::vector<Node*>& nodes)
	{
		(void) nodes;
		Triangle* triangle = new Triangle(*points_[0], *tail_, *head_);
		
		map_.push_back(triangle);
		
		af_head_ = new Node(*triangle->GetPoint(1), *triangle);
		af_middle_ = new Node(*triangle->GetPoint(0), *triangle);
		af_tail_ = new Node(*triangle->GetPoint(2));
		front_ = new AdvancingFront(*af_head_, *af_tail_);
	
		af_head_->next = af_middle_;
		af_middle_->next = af_tail_;
		af_middle_->prev = af_head_;
		af_tail_->prev = af_middle_;
	}
	
	void SweepLine::RemoveNode(Node* node)
	{
		delete node;
	}
	
	void SweepLine::MapTriangleToNodes(Triangle& t)
	{
		for (int i = 0; i < 3; i++)
		{
			if (!t.GetNeighbor(i))
			{
				Node* n = front_->LocatePoint(t.PointCW(*t.GetPoint(i)));
				if (n)
					n->triangle = &t;
			}
		}
	}
	
	void SweepLine::RemoveFromMap(Triangle* triangle)
	{
		map_.remove(triangle);
	}
	
	void SweepLine::MeshClean(Triangle& triangle)
	{
		std::vector<Triangle *> triangles;
		triangles.push_back(&triangle);
		
		while(!triangles.empty())
		{
			Triangle *t = triangles.back();
			triangles.pop_back();
			
			if (t != NULL && !t->IsInterior())
			{
				t->IsInterior(true);
				triangles_.push_back(t);
				for (int i = 0; i < 3; i++)
				{
					if (!t->constrained_edge[i])
						triangles.push_back(t->GetNeighbor(i));
				}
			}
		}
	}
	
	SweepLine::~SweepLine()
	{
		delete head_;
		delete tail_;
		delete front_;
		delete af_head_;
		delete af_middle_;
		delete af_tail_;
		
		typedef std::list<Triangle*> type_list;
		
		for(type_list::iterator iter = map_.begin(); iter != map_.end(); ++iter)
		{
			Triangle* ptr = *iter;
			delete ptr;
		}
		
		for(unsigned int i = 0; i < edge_list.size(); i++)
		{
			delete edge_list[i];
		}
	}
}
