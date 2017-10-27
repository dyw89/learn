#include "stdafx.h"
#include "cdt.h"

namespace cdt
{
	CDT::CDT()
	{
	}
	
	CDT::CDT(const std::vector<Point*>& polyline)
	{
		sweep_context_ = new SweepLine(polyline);
		sweep_ = new Sweep;
	}
	
	void CDT::AddConstraint(const std::vector<Point*>& polyline)
	{
		sweep_context_ = new SweepLine(polyline);
		sweep_ = new Sweep;
	}
	
	void CDT::AddHole(const std::vector<Point*>& polyline)
	{
		sweep_context_->AddHole(polyline);
	}

	void CDT::AddPoint(Point* point)
	{
		sweep_context_->AddPoint(point);
	}
	
	void CDT::Triangulate()
	{
		sweep_->Triangulate(*sweep_context_);
	}
	
	std::vector<cdt::Triangle*> CDT::GetTriangles() const
	{
		return sweep_context_->GetTriangles();
	}
	
	std::list<cdt::Triangle*> CDT::GetMap() const
	{
		return sweep_context_->GetMap();
	}
	
	CDT::~CDT()
	{
		delete sweep_context_;
		delete sweep_;
	}
}