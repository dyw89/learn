#ifndef CDT_H
#define CDT_H

#include "advancing_front.h"
#include "sweep_context.h"
#include "sweep.h"
#include "export.h"

namespace poly2tri
{
	class POLY2TRI_DLL CDT
	{
	public:		
		CDT(const std::vector<Point*>& polyline);
		CDT(void);
		
		~CDT();
		
		void AddHole(const std::vector<Point*>& polyline);		
		void AddConstraint(const std::vector<Point*>& polyline);		
		void AddPoint(Point* point);
		
		void Triangulate();
		
		std::vector<Triangle*> GetTriangles()const;		
		std::list<Triangle*> GetMap()const;
	
	private:		
		SweepContext* sweep_context_;
		Sweep* sweep_;
	};
}

#endif