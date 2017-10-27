#include "stdafx.h"
#include "cdt.h"

namespace cdt 
{
	AdvancingFront::AdvancingFront(Node& head, Node& tail)
	{
		head_ = &head;
		tail_ = &tail;
		search_node_ = &head;
	}
	
	Node* AdvancingFront::LocateNode(double x)
	{
		Node* node = search_node_;
		if (x < node->value)
		{
			while ((node = node->prev) != NULL)
			{
				if (x >= node->value)
				{
					search_node_ = node;
					return node;
				}
			}
		}
		else
		{
			while ((node = node->next) != NULL)
			{
				if (x < node->value)
				{
					search_node_ = node->prev;
					return node->prev;
				}
			}
		}
		return NULL;
	}
	
	Node* AdvancingFront::FindSearchNode(double x)
	{
		(void)x;
		return search_node_;
	}
	
	Node* AdvancingFront::LocatePoint(const Point* point)
	{
		const double px = point->xyz.x();
		Node* node = FindSearchNode(px);
		const double nx = node->point->xyz.x();
		
		if (fabs(px - nx) < 0.0000001)
		{
			if (point != node->point) 
			{
				if (point == node->prev->point)
				{
					node = node->prev;
				}
				else if (point == node->next->point)
				{
					node = node->next;
				}
				else
				{
					assert(0);
				}
			}
		}
		else if (px < nx)
		{
			while ((node = node->prev) != NULL)
			{
				if (point == node->point)
				{
					break;
				}
			}
		}
		else
		{
			while ((node = node->next) != NULL)
			{
				if (point == node->point)
					break;
			}
		}
		if(node) search_node_ = node;
		return node;
	}
	
	AdvancingFront::~AdvancingFront()
	{
	}
}