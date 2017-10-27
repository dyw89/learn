#ifndef CDT_H
#define CDT_H

#include <vector>
#include "cdt.h"
#include "shapes.h"
namespace cdt
{
	const double kAlpha = 0.3;
	struct Point;
	class Triangle;
	class AdvancingFront;
	class SweepLine;
	class Sweep;

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
		SweepLine* sweep_context_;
		Sweep* sweep_;
	};
	
	struct Node
	{
		Point* point;
		Triangle* triangle;

		Node* next;
		Node* prev;

		double value;

		Node(Point& p) : point(&p), triangle(NULL), next(NULL), prev(NULL), value(p.xyz.x())
		{
		}

		Node(Point& p, Triangle& t) : point(&p), triangle(&t), next(NULL), prev(NULL), value(p.xyz.x())
		{
		}
	};

	class AdvancingFront
	{
	public:
		AdvancingFront(Node& head, Node& tail);

		~AdvancingFront();

		Node* head()const;
		void set_head(Node* node);
		Node* tail()const;
		void set_tail(Node* node);
		Node* search()const;
		void set_search(Node* node);

		Node* LocateNode(double x);
		Node* LocatePoint(const Point* point);

	private:

		Node* head_, *tail_, *search_node_;

		Node* FindSearchNode(double x);
	};

	inline Node* AdvancingFront::head()const
	{
		return head_;
	}

	inline void AdvancingFront::set_head(Node* node)
	{
		head_ = node;
	}

	inline Node* AdvancingFront::tail()const
	{
		return tail_;
	}

	inline void AdvancingFront::set_tail(Node* node)
	{
		tail_ = node;
	}

	inline Node* AdvancingFront::search()const
	{
		return search_node_;
	}

	inline void AdvancingFront::set_search(Node* node)
	{
		search_node_ = node;
	}

	class SweepLine;
	struct Node;
	struct Point;
	struct Edge;
	class Triangle;

	class Sweep
	{
	public:
		void Triangulate(SweepLine& tcx);
		~Sweep();

	private:
		void SweepPoints(SweepLine& tcx);
		Node& PointEvent(SweepLine& tcx, Point& point);
		void EdgeEvent(SweepLine& tcx, Edge* edge, Node* node);
		void EdgeEvent(SweepLine& tcx, Point& ep, Point& eq, Triangle* triangle, Point& point);
		Node& NewFrontTriangle(SweepLine& tcx, Point& point, Node& node);
		void Fill(SweepLine& tcx, Node& node);
		bool Legalize(SweepLine& tcx, Triangle& t);

		bool Incircle(const Point& pa, const Point& pb, const Point& pc, const Point& pd) const;
		void RotateTrianglePair(Triangle& t, Point& p, Triangle& ot, Point& op) const;

		void FillAdvancingFront(SweepLine& tcx, Node& n);

		bool LargeHole_DontFill(const Node* node) const;
		bool AngleExceeds90Degrees(const Point* origin, const Point* pa, const Point* pb) const;
		bool AngleExceedsPlus90DegreesOrIsNegative(const Point* origin, const Point* pa, const Point* pb) const;
		double Angle(const Point* origin, const Point* pa, const Point* pb) const;

		double HoleAngle(const Node& node) const;
		double BasinAngle(const Node& node) const;

		void FillBasin(SweepLine& tcx, Node& node);
		void FillBasinReq(SweepLine& tcx, Node* node);
		bool IsShallow(SweepLine& tcx, Node& node)const;
		bool IsEdgeSideOfTriangle(Triangle& triangle, Point& ep, Point& eq)const;
		void FillEdgeEvent(SweepLine& tcx, Edge* edge, Node* node);
		void FillRightAboveEdgeEvent(SweepLine& tcx, Edge* edge, Node* node);
		void FillRightBelowEdgeEvent(SweepLine& tcx, Edge* edge, Node& node);
		void FillRightConcaveEdgeEvent(SweepLine& tcx, Edge* edge, Node& node);
		void FillRightConvexEdgeEvent(SweepLine& tcx, Edge* edge, Node& node);
		void FillLeftAboveEdgeEvent(SweepLine& tcx, Edge* edge, Node* node);
		void FillLeftBelowEdgeEvent(SweepLine& tcx, Edge* edge, Node& node);
		void FillLeftConcaveEdgeEvent(SweepLine& tcx, Edge* edge, Node& node);
		void FillLeftConvexEdgeEvent(SweepLine& tcx, Edge* edge, Node& node);
		void FlipEdgeEvent(SweepLine& tcx, Point& ep, Point& eq, Triangle* t, Point& p);

		Triangle& NextFlipTriangle(SweepLine& tcx, int o, Triangle&  t, Triangle& ot, Point& p, Point& op);
		Point& NextFlipPoint(Point& ep, Point& eq, Triangle& ot, Point& op)const;

		void FlipScanEdgeEvent(SweepLine& tcx, Point& ep, Point& eq, Triangle& flip_triangle, Triangle& t, Point& p);
		void FinalizationPolygon(SweepLine& tcx);
		std::vector<Node*> nodes_;
	};

	class SweepLine
	{
	public:
		SweepLine(const std::vector<Point*>& polyline);
		~SweepLine();

		void set_head(Point* p1);
		Point* head() const;
		void set_tail(Point* p1);
		Point* tail() const;
		size_t point_count() const;
		Node& LocateNode(const Point& point);
		void RemoveNode(Node* node);
		void CreateAdvancingFront(const std::vector<Node*>& nodes);
		void MapTriangleToNodes(Triangle& t);
		void AddToMap(Triangle* triangle);
		Point* GetPoint(size_t index)const;
		Point* GetPoints()const;

		void RemoveFromMap(Triangle* triangle);
		void AddHole(const std::vector<Point*>& polyline);
		void AddPoint(Point* point);

		AdvancingFront* front() const;

		void MeshClean(Triangle& triangle);

		std::vector<Triangle*> &GetTriangles();
		std::list<Triangle*> &GetMap();
		std::vector<Edge*> edge_list;

		struct Basin
		{
			Node* left_node;
			Node* bottom_node;
			Node* right_node;
			double width;
			bool left_highest;

			Basin() : left_node(NULL), bottom_node(NULL), right_node(NULL), width(0.0), left_highest(false)
			{
			}

			void Clear()
			{
				left_node = NULL;
				bottom_node = NULL;
				right_node = NULL;
				width = 0.0;
				left_highest = false;
			}
		};

		struct EdgeEvent
		{
			Edge* constrained_edge;
			bool right;

			EdgeEvent() : constrained_edge(NULL), right(false)
			{
			}
		};

		Basin basin;
		EdgeEvent edge_event;

	private:

		friend class Sweep;

		std::vector<Triangle*> triangles_;
		std::list<Triangle*> map_;
		std::vector<Point*> points_;

		AdvancingFront* front_;
		Point* head_;
		Point* tail_;

		Node *af_head_, *af_middle_, *af_tail_;

		void InitTriangulation();
		void InitEdges(const std::vector<Point*>& polyline);
	};

	inline AdvancingFront* SweepLine::front() const
	{
		return front_;
	}

	inline size_t SweepLine::point_count() const
	{
		return points_.size();
	}

	inline void SweepLine::set_head(Point* p1)
	{
		head_ = p1;
	}

	inline Point* SweepLine::head() const
	{
		return head_;
	}

	inline void SweepLine::set_tail(Point* p1)
	{
		tail_ = p1;
	}

	inline Point* SweepLine::tail() const
	{
		return tail_;
	}
}

#endif