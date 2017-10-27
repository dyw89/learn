//修改，在原pol2tri基础上修改直接使用osg结构

#include <cstdlib>
#include <time.h>
#include <fstream>
#include <string>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <iostream>
#include <vector>
#include <list>
#include <iomanip>
#include <math.h>

#include <osg/Array>
#include <osg/Group>   
#include <osg/Geode> 
#include <osg/Material>
#include <osgViewer/Viewer>   
#include <osg/ShapeDrawable> 
#include <osgGA/TrackballManipulator>
#include <osgViewer/ViewerEventHandlers> 
#include <osgUtil/IntersectionVisitor>
#include <osgUtil/PlaneIntersector>
#include <osg/CullFace> 
#include <osg/PolygonMode>
#include <osgUtil/DelaunayTriangulator>
#include <osgGA/StateSetManipulator>
#include <osgDB/ReadFile>  
#include <osgDB/WriteFile>


//using namespace std;
using namespace std;
#include "triangulation.h" //polygpn 2 triangle

//using namespace p2t; 

double StringToDouble(const std::string& s);
double Random(double(*fun)(double), double xmin, double xmax);
double Fun(double x); //用于产生随机数
void Writedata(std::string path1, const std::vector<cdt::Triangle*>& tri);
//std::string outpath = "E://练习//test//test//testbed//result.txt";
/// Dude hole examples
std::vector<cdt::Point*> CreateHeadHole();
std::vector<cdt::Point*> CreateChestHole();


/// Screen center x
double cx = 0.0;
/// Screen center y
double cy = 0.0;

/// Constrained triangles 约束三角形
std::vector<cdt::Triangle*> triangles;
/// Triangle map
std::list<cdt::Triangle*> map1;

/// Polylines
std::vector< std::vector<cdt::Point*> > polylines;

/// Create a random distribution of points?
bool random_distribution = false;

//释放内存
template <class C> void FreeClear(C & cntr)
{
	for (typename C::iterator it = cntr.begin(); it != cntr.end(); ++it)
	{
		if (*it != NULL)
		{
			delete *it;
			*it = NULL;
		}
	}
	cntr.clear();
}

struct Point
{
	int type;
	int index;
	int flag;//为适应带岛屿多边形的三角剖分
	osg::Vec3d xyz;
	Point(){}
	~Point(){}

	Point(const osg::Vec3d& inxyz)
	{
		type = 0;
		index = 0;
		flag = 0;
		xyz = inxyz;
	}

	Point(const int& intype, const osg::Vec3d& inxyz)
	{
		type = intype;
		xyz = inxyz;
	}

	Point(const int& intype, const int& inindex, const osg::Vec3d& inxyz)
	{
		type = intype;
		index = inindex;
		xyz = inxyz;
	}

	Point(const int& intype, const int& inindex,const int&inflag, const osg::Vec3d& inxyz)
	{
		type = intype;
		index = inindex;
		flag = inflag;
		xyz = inxyz;
	}

	/*bool operator == (const Point& inp)
	{
		if (fabs(xyz[0] - inp.xyz[0]) <= 0.000001&&fabs(xyz[1] - inp.xyz[1]) <= 0.000001&&fabs(xyz[2] - inp.xyz[2]) <= 0.000001)
			return true;
		return false;
	}

	bool operator != (const Point& inp)
	{
		if (fabs(xyz[0] - inp.xyz[0]) >= 0.000001&&fabs(xyz[1] - inp.xyz[1]) >= 0.000001&&fabs(xyz[2] - inp.xyz[2]) >= 0.000001)
			return true;
		return false;
	}*/

	bool operator != (const Point& inp) const
	{
		if (fabs(xyz[0] - inp.xyz[0]) >= 0.000001&&fabs(xyz[1] - inp.xyz[1]) >= 0.000001&&fabs(xyz[2] - inp.xyz[2]) >= 0.000001)
			return true;
		return false;
	}

	bool operator == (const Point& inp) const
	{
		if (fabs(xyz[0] - inp.xyz[0]) <= 0.000001&&fabs(xyz[1] - inp.xyz[1]) <= 0.000001&&fabs(xyz[2] - inp.xyz[2]) <= 0.000001)
			return true;
		return false;
	}
	void operator = (const Point& inp)
	{
		xyz = inp.xyz;
		index = inp.index;
		type = inp.type;
		flag = inp.flag;
	}
};
struct Area
{
	std::vector<Point> poly;//区域外边界
	std::vector<std::vector<Point>> hole;//区域内挖空区域
	Area(){}
	~Area(){}
	Area(vector<Point>& inpoly)
	{
		poly.insert(poly.begin(), inpoly.begin(), inpoly.end());
	}
	void addarea(vector<Point>& inpoly)
	{
		poly.insert(poly.begin(), inpoly.begin(), inpoly.end());
	}
	void addhole(vector<Point>& inhole)
	{
		hole.push_back(inhole);
	}
};

struct PointIndex
{
	int type;
	int index;
	PointIndex(){}
	PointIndex(const int& intype, const int& inindex)
	{
		type = intype;
		index = inindex;
	}
	void operator=(const PointIndex& inp)
	{
		type = inp.type;
		index = inp.index;
	}
};
struct Triangle
{
	Point p1;
	Point p2;
	Point p3;
	Triangle(){}
	Triangle(const Point& inp1, const Point& inp2, const Point& inp3)
	{
		p1 = inp1;
		p2 = inp2;
		p3 = inp3;
	}
};
struct TriangleIndex
{
	PointIndex p1;
	PointIndex p2;
	PointIndex p3;
	TriangleIndex(){}
	TriangleIndex(const PointIndex& inp1, const PointIndex& inp2, const PointIndex& inp3)
	{
		p1 = inp1;
		p2 = inp2;
		p3 = inp3;
	}
};

void SplitAreaToTriangle(const Area& thearea, vector<TriangleIndex>& splitresult); //多边形三角化，可以带洞
void Triangulated(const vector<Point>& outline, const vector<Point>& points, const vector<vector<Point>>& holes, bool flag, vector<TriangleIndex>& splitresult); //给定边界、点集、洞，进行三角化，flag指示是否对洞填挖
void Triangulated(const vector<Point>& outline, const vector<Point>& points, const vector<vector<Point>>& holes, bool flag, vector<Triangle>& splitresult);
void Triangulated(const vector<Point>& points, vector<TriangleIndex>& splitresult); //离散点集delaunay三角化
void Triangulated(const vector<Point>& points, vector<Triangle>& splitresult);
void Change(const vector<vector<cdt::Point*>>& pol, Area& pol1);
osg::Geode* ShowResult(const std::vector<cdt::Triangle*>& tri);
osg::Geode* ShowResult(const std::vector<Triangle>& tri);
osg::Geode* ShowResult1(const std::vector<cdt::Triangle*>& tri);
osg::Geode* ShowResult(const std::list<cdt::Triangle*>& tri);
osg::Geode* ShowResult(const vector<TriangleIndex>& tri, const vector<Point>& point);
bool PointinPoly(std::vector<osg::Vec3d>polygon, osg::Vec3d p);

int main(int argc, char* argv[])
{
	Area polys;
	vector<Point> pointlist;
	vector<Point> points;
	vector<Point> outline;
	vector<vector<Point>> holes;
	vector<TriangleIndex> result, result1;
	vector<Triangle> result2;
	int num_points = 0;
	double max, min;
	double zoom;
	std::string inpath = "G://交规院//work//Main//src//Poly2tritest//data//2.dat";;
	//inpath = "G://交规院//work//Main//src//Poly2tritest//data//2.dat";

	/*if (argc != 5)
	{
		cout << "-== USAGE ==-" << endl;
		cout << "Load Data File: p2t filename center_x center_y zoom" << endl;
		cout << "Example: ./build/p2t testbed/data/dude.dat 500 500 1" << endl;
		return 1;
	}*/

	std::string s1(inpath);
	if (s1.find("random", 0) != string::npos)
	//if (string(argv[1]) == "random")
	{
		//num_points = atoi(argv[2]);
		num_points = 500;
		random_distribution = true;
		char* pEnd;
		/*max = strtod(argv[3], &pEnd);*/
		max = 500;
		min = -max;
		cx = cy = 0;
		/*zoom = atof(argv[4]);*/
		zoom = 1;
	}
	else
	{
		/*zoom = atof(argv[4]);
		cx = atof(argv[2]);
		cy = atof(argv[3]);*/
		zoom = 1;
		cx = 500;
		cy = 500;
	}

	std::vector<cdt::Point*> polyline;

	if (random_distribution)
	{
		// Create a simple bounding box
		polyline.push_back(new cdt::Point(min, min));
		polyline.push_back(new cdt::Point(min, max));
		polyline.push_back(new cdt::Point(max, max));
		polyline.push_back(new cdt::Point(max, min));

		outline.push_back(Point(0, 0, osg::Vec3d(min, min, 0)));
		outline.push_back(Point(0, 0, osg::Vec3d(min, max, 0)));
		outline.push_back(Point(0, 0, osg::Vec3d(max, max, 0)));
		outline.push_back(Point(0, 0, osg::Vec3d(max, min, 0)));
	}
	else
	{
		// 从文件加载数据
		// Parse and tokenize data file
		string line;
		/*ifstream myfile(argv[1]);*/
		ifstream myfile(inpath);
		if (myfile.is_open())
		{
			int i = 0;
			int flag = 0;
			vector<Point>ho;
			while (!myfile.eof())
			{
				getline(myfile, line);
				if (flag != 2 && line.size() == 0)
				{
					break;
				}
				if (line.find("outline", 0) != string::npos)
				{
					flag = 1;
					getline(myfile, line);
				}	
				else if (line.find("hole", 0) != string::npos)
				{
					flag = 2;
					getline(myfile, line);
					polylines.push_back(polyline);
					polyline.clear();
				}
				else if (line.find("point", 0) != string::npos)
				{
					flag = 3;
					getline(myfile, line);
				}
				if (flag == 1)
				{
					istringstream iss(line);
					std::vector<string> tokens;
					copy(istream_iterator<string>(iss), istream_iterator<string>(), back_inserter<std::vector<string> >(tokens));
					if (tokens.size() == 2)
					{
						double x = StringToDouble(tokens[0]);
						double y = StringToDouble(tokens[1]);
						polyline.push_back(new cdt::Point(0, i++, osg::Vec3d(x, y, 0)));
						outline.push_back(Point(0, i - 1, osg::Vec3d(x, y, 0)));
					}
					else
					{
						double x = StringToDouble(tokens[0]);
						double y = StringToDouble(tokens[1]);
						double z = StringToDouble(tokens[2]);
						polyline.push_back(new cdt::Point(0, i++, osg::Vec3d(x, y, z)));
						outline.push_back(Point(0, i - 1, osg::Vec3d(x, y, z)));
					}
					num_points++;
				}	
				else if (flag == 2)
				{
					if (line.size()==0)
					{
						holes.push_back(ho);
						ho.clear();
						polylines.push_back(polyline);
						polyline.clear();
						continue;
					}
					istringstream iss(line);
					std::vector<string> tokens;
					copy(istream_iterator<string>(iss), istream_iterator<string>(), back_inserter<std::vector<string> >(tokens));
					if (tokens.size() == 2)
					{
						double x = StringToDouble(tokens[0]);
						double y = StringToDouble(tokens[1]);
						polyline.push_back(new cdt::Point(0, i++, osg::Vec3d(x, y, 0)));
						ho.push_back(Point(0, i - 1, osg::Vec3d(x, y, 0)));
					}
					else
					{
						double x = StringToDouble(tokens[0]);
						double y = StringToDouble(tokens[1]);
						double z = StringToDouble(tokens[2]);
						polyline.push_back(new cdt::Point(0, i++, osg::Vec3d(x, y, z)));
						ho.push_back(Point(0, i - 1, osg::Vec3d(x, y, z)));
					}
					num_points++;
				}
				else if (flag == 3)
				{
					istringstream iss(line);
					std::vector<string> tokens;
					copy(istream_iterator<string>(iss), istream_iterator<string>(), back_inserter<std::vector<string> >(tokens));
					if (tokens.size() == 2)
					{
						double x = StringToDouble(tokens[0]);
						double y = StringToDouble(tokens[1]);
						//polyline.push_back(new trigulate::Point(0, i++, osg::Vec3d(x, y, 0)));
						pointlist.push_back(Point(0, i++, osg::Vec3d(x, y, 0)));
					}
					else
					{
						double x = StringToDouble(tokens[0]);
						double y = StringToDouble(tokens[1]);
						double z = StringToDouble(tokens[2]);
						//polyline.push_back(new trigulate::Point(0, i++, osg::Vec3d(x, y, z)));
						pointlist.push_back(Point(0, i++, osg::Vec3d(x, y, z)));
					}
					num_points++;
				}
			}
			holes.push_back(ho);
			polylines.push_back(polyline);
			myfile.close();
		}
		else
		{
			cout << "File not opened" << endl;
		}
	}

	/*cout << "Number of constrained edges = " << polyline.size() << endl;
	polylines.push_back(polyline);
		
	trigulate::CDT cdt = trigulate::CDT();
	cdt.AddConstraint(polylines[0]);*/


	//string s(argv[1]);
	//string s(inpath);
	//if (s.find("dude.dat", 0) != string::npos)
	//{
	//	// Add head hole
	//	std::vector<trigulate::Point*> head_hole = CreateHeadHole();
	//	num_points += head_hole.size();
	//	cdt.AddHole(head_hole);
	//	polylines.push_back(head_hole);
	//	
	//	vector<Point> temp;
	//	for (int i = 0; i < head_hole.size(); i++)
	//	{
	//		temp.push_back(Point(head_hole[i]->type, head_hole[i]->index, osg::Vec3d(head_hole[i]->xyz.x(), head_hole[i]->xyz.y(), head_hole[i]->xyz.z())));
	//	}
	//	holes.push_back(temp);
	//	// Add chest hole
	//	std::vector<trigulate::Point*> chest_hole = CreateChestHole();
	//	num_points += chest_hole.size();
	//	cdt.AddHole(chest_hole);
	//	polylines.push_back(chest_hole);

	//	temp.clear();
	//	for (int i = 0; i < chest_hole.size(); i++)
	//	{
	//		temp.push_back(Point(chest_hole[i]->type, chest_hole[i]->index, osg::Vec3d(chest_hole[i]->xyz.x(), chest_hole[i]->xyz.y(), chest_hole[i]->xyz.z())));
	//	}
	//	holes.push_back(temp);
	//}
	//else if (random_distribution)
	//{
	//	max -= (1e-4);
	//	min += (1e-4);
	//	for (int i = 0; i < num_points; i++)
	//	{
	//		double x = Random(Fun, min, max);
	//		double y = Random(Fun, min, max);
	//		cdt.AddPoint(new trigulate::Point(i, 0, osg::Vec3(x, y, 0)));
	//		pointlist.push_back(Point(0, i, osg::Vec3d(x, y, 0)));
	//	}
	//}
	//else if (s.find("test3.dat", 0) != string::npos)
	//{
	//	cdt.AddPoint(new trigulate::Point(0, 0, osg::Vec3d(1, 1, 0)));
	//}
	//else if (s.find("test4.dat", 0) != string::npos)
	//{
	//	int n = num_points;

	//	cdt.AddPoint(new trigulate::Point(0, 0, osg::Vec3d(168.43408179600, -247.08492381700, 0)));
	//	cdt.AddPoint(new trigulate::Point(0, 0, osg::Vec3d(265.65044105900, -266.84434643100, 0)));
	//	cdt.AddPoint(new trigulate::Point(0, 0, osg::Vec3d(354.96303127500, -245.50417000800, 0)));
	//	cdt.AddPoint(new trigulate::Point(0, 0, osg::Vec3d(416.61242983200, -192.54891740200, 0)));
	//	cdt.AddPoint(new trigulate::Point(0, 0, osg::Vec3d(467.98692862900, -161.72421812300, 0)));
	//	cdt.AddPoint(new trigulate::Point(0, 0, osg::Vec3d(584.96271050500, -128.52838813200, 0)));
	//	cdt.AddPoint(new trigulate::Point(0, 0, osg::Vec3d(528.84595028100, -61.34635124300, 0)));
	//	cdt.AddPoint(new trigulate::Point(0, 0, osg::Vec3d(418.98356054500, -36.84466720130, 0)));
	//	cdt.AddPoint(new trigulate::Point(0, 0, osg::Vec3d(355.75340818000, -172.78949478700, 0)));
	//	cdt.AddPoint(new trigulate::Point(0, 0, osg::Vec3d(279.87722534100, -126.94763432200, 0)));
	//	cdt.AddPoint(new trigulate::Point(0, 0, osg::Vec3d(196.09727345600, -145.91668003200, 0)));
	//	cdt.AddPoint(new trigulate::Point(0, 0, osg::Vec3d(113.10769847600, -175.16062550100, 0)));
	//	cdt.AddPoint(new trigulate::Point(0, 0, osg::Vec3d(62.52357658380, -141.96479550900, 0)));
	//	cdt.AddPoint(new trigulate::Point(0, 0, osg::Vec3d(200.04915797900, -86.63841218930, 0)));
	//	cdt.AddPoint(new trigulate::Point(0, 0, osg::Vec3d(85.44450681640, -103.23632718500, 0)));

	//	pointlist.push_back(Point(0, n++, osg::Vec3d(168.43408179600, -247.08492381700, 0)));
	//	pointlist.push_back(Point(0, n++, osg::Vec3d(265.65044105900, -266.84434643100, 0)));
	//	pointlist.push_back(Point(0, n++, osg::Vec3d(354.96303127500, -245.50417000800, 0)));
	//	pointlist.push_back(Point(0, n++, osg::Vec3d(416.61242983200, -192.54891740200, 0)));
	//	pointlist.push_back(Point(0, n++, osg::Vec3d(467.98692862900, -161.72421812300, 0)));
	//	pointlist.push_back(Point(0, n++, osg::Vec3d(584.96271050500, -128.52838813200, 0)));
	//	pointlist.push_back(Point(0, n++, osg::Vec3d(528.84595028100, -61.34635124300, 0)));
	//	pointlist.push_back(Point(0, n++, osg::Vec3d(418.98356054500, -36.84466720130, 0)));
	//	pointlist.push_back(Point(0, n++, osg::Vec3d(355.75340818000, -172.78949478700, 0)));
	//	pointlist.push_back(Point(0, n++, osg::Vec3d(279.87722534100, -126.94763432200, 0)));
	//	pointlist.push_back(Point(0, n++, osg::Vec3d(196.09727345600, -145.91668003200, 0)));
	//	pointlist.push_back(Point(0, n++, osg::Vec3d(113.10769847600, -175.16062550100, 0)));
	//	pointlist.push_back(Point(0, n++, osg::Vec3d(62.52357658380, -141.96479550900, 0)));
	//	pointlist.push_back(Point(0, n++, osg::Vec3d(200.04915797900, -86.63841218930, 0)));
	//	pointlist.push_back(Point(0, n++, osg::Vec3d(85.44450681640, -103.23632718500, 0)));
	//}
	//else if (s.find("test1.dat", 0) != string::npos)
	//{
	//	cdt.AddPoint(new trigulate::Point(0, 0, osg::Vec3d(2, 1, 0)));
	//}

	//
	//cdt.Triangulate();
	//triangles = cdt.GetTriangles();
	//triangles.at(1)->DebugPrint(); //打印三角形
	//
	//map1 = cdt.GetMap();
	//Writedata(outpath, triangles);

	//Change(polylines,polys);
	//SplitAreaToTriangle(polys,result);

	for (int i = 0; i < polylines.size(); i++)
	{
		for (int j = 0; j < polylines[i].size(); j++)
		{
			points.push_back(Point(polylines[i][j]->type,polylines[i][j]->index,polylines[i][j]->xyz));
		}
	}
	for (int i = 0; i < pointlist.size(); i++)
	{
		points.push_back(pointlist[i]);
	}

	double a = 1.8839097e-15, b = 1.5699247e-16;
	double c = a - b;
	if (a > b)
	{
		cout << "true" << endl;
	}
    std:; cout << "Number of points = " << num_points << endl;
	cout << "Number of triangles = " << triangles.size() << endl;
	Triangulated(outline, pointlist, holes, true, result);
	//osg::ref_ptr<osg::Node> geode = ShowResult(triangles);  //创建节点
	osg::ref_ptr<osg::Node> geode2 = ShowResult(result2);  //创建节点
	//osg::ref_ptr<osg::Node> geode1 = ShowResult(map1);  //创建节点
	osg::ref_ptr<osg::Node> geode = ShowResult(result,points);
	osg::ref_ptr<osg::Group> root = new osg::Group; //创建场景组节点
	//root->addChild(geode2.get());
	root->addChild(geode.get()); //添加到场景
	//root->addChild(geode1.get());
	osgViewer::Viewer* viewer = new osgViewer::Viewer();  //创建场景

	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
	traits->x = 40;        //x偏移
	traits->y = 40;       //y偏移
	traits->width = 600;
	traits->height = 480;
	traits->windowDecoration = true;   //是否支持窗口一些装饰属性..如最大化 最小化等
	traits->doubleBuffer = true;      //是否支持双缓存
	traits->sharedContext = 0;       //共享GraphicsContext

	osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());

	osg::ref_ptr<osg::Camera> camera = new osg::Camera;
	camera->setGraphicsContext(gc.get());
	camera->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));
	GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
	camera->setDrawBuffer(buffer);
	camera->setReadBuffer(buffer);


	// add this slave camera to the viewer, with a shift left of the projection matrix
	viewer->addSlave(camera.get());

	viewer->setSceneData(root.get());

	//添加状态事件
	viewer->addEventHandler(new osgGA::StateSetManipulator(viewer->getCamera()->getOrCreateStateSet()));
	//窗口大小变化事件
	viewer->addEventHandler(new osgViewer::WindowSizeHandler);
	//添加一些常用状态设置
	viewer->addEventHandler(new osgViewer::StatsHandler);
	viewer->realize();
	viewer->run();

	// Cleanup 释放内存
	//delete cdt;

	// Free points 释放点数据
	for (int i = 0; i < polylines.size(); i++)
	{
		std::vector<cdt::Point*> poly = polylines[i];
		FreeClear(poly);
	}
	
	return 0;
}


std::vector<cdt::Point*> CreateHeadHole() 
{
	std::vector<cdt::Point*> head_hole;
	int index1 = 0;
	for (int i = 0; i < polylines.size(); i++)
	{
		index1 += polylines[i].size();
	}
	head_hole.push_back(new cdt::Point(0, index1++, osg::Vec3d(325, 437, 0)));
	head_hole.push_back(new cdt::Point(0, index1++, osg::Vec3d(320, 423, 0)));
	head_hole.push_back(new cdt::Point(0, index1++, osg::Vec3d(329, 413, 0)));
	head_hole.push_back(new cdt::Point(0, index1++, osg::Vec3d(332, 423, 0)));

	return head_hole;
}

std::vector<cdt::Point*> CreateChestHole() 
{
	std::vector<cdt::Point*> chest_hole;
	int index1 = 0;
	for (int i = 0; i < polylines.size(); i++)
	{
		index1 += polylines[i].size();
	}
	chest_hole.push_back(new cdt::Point(0, index1++, osg::Vec3d(320.72342, 480, 0)));
	chest_hole.push_back(new cdt::Point(0, index1++, osg::Vec3d(338.90617, 465.96863, 0)));
	chest_hole.push_back(new cdt::Point(0, index1++, osg::Vec3d(347.99754, 480.61584, 0)));
	chest_hole.push_back(new cdt::Point(0, index1++, osg::Vec3d(329.8148, 510.41534, 0)));
	chest_hole.push_back(new cdt::Point(0, index1++, osg::Vec3d(339.91632, 480.11077, 0)));
	chest_hole.push_back(new cdt::Point(0, index1++, osg::Vec3d(334.86556, 478.09046, 0)));

	return chest_hole;
}

double StringToDouble(const std::string& s)
{
	std::istringstream i(s);
	double x;
	if (!(i >> x))
		return 0;
	return x;
}

double Fun(double x)
{
	return 2.5 + sin(10 * x) / x;
}

double Random(double(*fun)(double), double xmin = 0, double xmax = 1)
{
	static double(*Fun)(double) = NULL, YMin, YMax;
	static bool First = true;

	if (First)
	{
		First = false;
		srand((unsigned)time(NULL));
	}

	if (fun != Fun)
	{
		Fun = fun;
		YMin = 0, YMax = Fun(xmin);
		for (int iX = 1; iX < RAND_MAX; iX++)
		{
			double X = xmin + (xmax - xmin) * iX / RAND_MAX;
			double Y = Fun(X);
			YMax = Y > YMax ? Y : YMax;
		}
	}

	double X = xmin + (xmax - xmin) * rand() / RAND_MAX;
	double Y = YMin + (YMax - YMin) * rand() / RAND_MAX;

	return Y < fun(X) ? X : Random(Fun, xmin, xmax);
}

osg::Geode* ShowResult(const std::vector<cdt::Triangle*>& tri)
{
	osg::Geode* geode = new osg::Geode;

	osg::Geometry* triangle = new osg::Geometry;
	geode->addDrawable(triangle);

	osg::Vec3Array* Tvertex = new osg::Vec3Array;
	triangle->setVertexArray(Tvertex);

	osg::Vec3Array* Tcolor = new osg::Vec3Array;
	triangle->setColorArray(Tcolor);
	triangle->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

	double num = 0;
	for (std::vector<cdt::Triangle*>::const_iterator it = tri.begin(); it != tri.end(); it++)
	{
		cdt::Triangle& t = **it;
		cdt::Point& a = *t.GetPoint(0);
		cdt::Point& b = *t.GetPoint(1);
		cdt::Point& c = *t.GetPoint(2);
		Tvertex->push_back(osg::Vec3(a.xyz.x(), a.xyz.y(), a.xyz.z()));
		Tvertex->push_back(osg::Vec3(b.xyz.x(), b.xyz.y(), b.xyz.z()));
		Tvertex->push_back(osg::Vec3(c.xyz.x(), c.xyz.y(), c.xyz.z()));

		Tcolor->push_back(osg::Vec3(0.0, 0.0, 1.0));
		Tcolor->push_back(osg::Vec3(0.0, 0.0, 1.0));
		Tcolor->push_back(osg::Vec3(0.0, 0.0, 1.0));
		num = num + 3;
	}

	/** set the normal*/
	osg::Vec3Array* Tnormal = new osg::Vec3Array;
	Tnormal->push_back(osg::Vec3(0.0, -1.0, 0.0));
	//triangle->setNormalArray(Tnormal);
	triangle->setNormalBinding(osg::Geometry::BIND_OVERALL);

	/** set the Primitive use GL_TRIANGLES*/
	for (int i = 0; i < num - 2; i += 3)
	{
		osg::PrimitiveSet* Tprimitive = new osg::DrawArrays(GL_TRIANGLES, i, 3);
		triangle->addPrimitiveSet(Tprimitive);
	}

	return geode;
}

osg::Geode* ShowResult(const std::vector<Triangle>& tri)
{
	osg::Geode* geode = new osg::Geode;

	osg::Geometry* triangle = new osg::Geometry;
	geode->addDrawable(triangle);

	osg::Vec3Array* Tvertex = new osg::Vec3Array;
	triangle->setVertexArray(Tvertex);

	osg::Vec3Array* Tcolor = new osg::Vec3Array;
	triangle->setColorArray(Tcolor);
	triangle->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

	double num = 0;
	for (std::vector<Triangle>::const_iterator it = tri.begin(); it != tri.end(); it++)
	{
		Tvertex->push_back(it->p1.xyz);
		Tvertex->push_back(it->p2.xyz);
		Tvertex->push_back(it->p3.xyz);

		Tcolor->push_back(osg::Vec3(0.0, 0.0, 1.0));
		Tcolor->push_back(osg::Vec3(0.0, 0.0, 1.0));
		Tcolor->push_back(osg::Vec3(0.0, 0.0, 1.0));
		num = num + 3;
	}

	/** set the normal*/
	osg::Vec3Array* Tnormal = new osg::Vec3Array;
	//Tnormal->push_back(osg::Vec3(0.0, -1.0, 0.0));
	//triangle->setNormalArray(Tnormal);
	triangle->setNormalBinding(osg::Geometry::BIND_OVERALL);

	/** set the Primitive use GL_TRIANGLES*/
	for (int i = 0; i < num - 2; i += 3)
	{
		osg::PrimitiveSet* Tprimitive = new osg::DrawArrays(GL_TRIANGLES, i, 3);
		triangle->addPrimitiveSet(Tprimitive);
	}

	return geode;
}

osg::Geode* ShowResult1(const std::vector<cdt::Triangle*>& tri)
{
	/**
	geode is a leaf node,It must contain some drawables..
	*/
	osg::Geode* geode = new osg::Geode;

	/** define a Geometry for draw a triangle.*/
	osg::Geometry* triangle = new osg::Geometry;
	geode->addDrawable(triangle);

	/** triangle Vertexs */
	osg::Vec3Array* Tvertex = new osg::Vec3Array;
	triangle->setVertexArray(Tvertex);

	osg::Vec3Array* Tcolor = new osg::Vec3Array;
	triangle->setColorArray(Tcolor);
	triangle->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

	double num = 0;
	for (std::vector<cdt::Triangle*>::const_iterator it = tri.begin(); it != tri.end(); it++)
	{
		cdt::Triangle& t = **it;
		cdt::Point& a = *t.GetPoint(0);
		cdt::Point& b = *t.GetPoint(1);
		cdt::Point& c = *t.GetPoint(2);
		Tvertex->push_back(osg::Vec3(a.xyz.x(), a.xyz.y(), a.xyz.z()));
		Tvertex->push_back(osg::Vec3(b.xyz.x(), b.xyz.y(), b.xyz.z()));
		Tvertex->push_back(osg::Vec3(c.xyz.x(), c.xyz.y(), c.xyz.z()));

		Tcolor->push_back(osg::Vec3(1.0, 1.0, 1.0));
		Tcolor->push_back(osg::Vec3(1.0, 1.0, 1.0));
		Tcolor->push_back(osg::Vec3(1.0, 1.0, 1.0));
		num = num + 3;
	}

	/** set the normal*/
	osg::Vec3Array* Tnormal = new osg::Vec3Array;
	Tnormal->push_back(osg::Vec3(0.0, -1.0, 0.0));
	//triangle->setNormalArray(Tnormal);
	triangle->setNormalBinding(osg::Geometry::BIND_OVERALL);

	/** set the Primitive use GL_TRIANGLES*/
	for (int i = 0; i < num - 2; i += 3)
	{
		osg::PrimitiveSet* Tprimitive = new osg::DrawArrays(GL_TRIANGLES, i, 3);
		triangle->addPrimitiveSet(Tprimitive);
	}

	return geode;
}

osg::Geode* ShowResult(const std::list<cdt::Triangle*>& tri)
{
	/**
	geode is a leaf node,It must contain some drawables..
	*/
	osg::Geode* geode = new osg::Geode;

	/** define a Geometry for draw a triangle.*/
	osg::Geometry* triangle = new osg::Geometry;
	geode->addDrawable(triangle);

	/** triangle Vertexs */
	osg::Vec3Array* Tvertex = new osg::Vec3Array;
	triangle->setVertexArray(Tvertex);

	osg::Vec3Array* Tcolor = new osg::Vec3Array;
	triangle->setColorArray(Tcolor);
	triangle->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

	double num = 0;
	for (std::list<cdt::Triangle*>::const_iterator it = tri.begin(); it != tri.end(); it++)
	{
		cdt::Triangle& t = **it;
		cdt::Point& a = *t.GetPoint(0);
		cdt::Point& b = *t.GetPoint(1);
		cdt::Point& c = *t.GetPoint(2);
		Tvertex->push_back(osg::Vec3(a.xyz.x(), a.xyz.y(), a.xyz.z()));
		Tvertex->push_back(osg::Vec3(b.xyz.x(), b.xyz.y(), b.xyz.z()));
		Tvertex->push_back(osg::Vec3(c.xyz.x(), c.xyz.y(), c.xyz.z()));

		Tcolor->push_back(osg::Vec3(0.0, 1.0, 0.0));
		Tcolor->push_back(osg::Vec3(0.0, 1.0, 0.0));
		Tcolor->push_back(osg::Vec3(0.0, 1.0, 0.0));
		num = num + 3;
	}

	/** set the normal*/
	osg::Vec3Array* Tnormal = new osg::Vec3Array;
	Tnormal->push_back(osg::Vec3(0.0, -1.0, 0.0));
	//triangle->setNormalArray(Tnormal);
	triangle->setNormalBinding(osg::Geometry::BIND_OVERALL);

	/** set the Primitive use GL_TRIANGLES*/
	for (int i = 0; i < num - 2; i += 3)
	{
		osg::PrimitiveSet* Tprimitive = new osg::DrawArrays(GL_TRIANGLES, i, 3);
		triangle->addPrimitiveSet(Tprimitive);
	}

	return geode;
}

osg::Geode* ShowResult(const vector<TriangleIndex>& tri, const vector<Point>& point)
{
	osg::Geode* geode = new osg::Geode;

	/** define a Geometry for draw a triangle.*/
	osg::Geometry* triangle = new osg::Geometry;
	geode->addDrawable(triangle);

	/** triangle Vertexs */
	osg::Vec3Array* Tvertex = new osg::Vec3Array;
	triangle->setVertexArray(Tvertex);

	osg::Vec3Array* Tcolor = new osg::Vec3Array;
	triangle->setColorArray(Tcolor);
	triangle->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

	double num = 0;
	for (vector<TriangleIndex>::const_iterator it = tri.begin(); it != tri.end(); it++)
	{
		Tvertex->push_back(point[it->p1.index].xyz);
		Tvertex->push_back(point[it->p2.index].xyz);
		Tvertex->push_back(point[it->p3.index].xyz);

		Tcolor->push_back(osg::Vec3(0.0, 0.0, 1.0));
		Tcolor->push_back(osg::Vec3(0.0, 0.0, 1.0));
		Tcolor->push_back(osg::Vec3(0.0, 0.0, 1.0));
		num = num + 3;
	}

	/** set the normal*/
	osg::Vec3Array* Tnormal = new osg::Vec3Array;
	Tnormal->push_back(osg::Vec3(0.0, -1.0, 0.0));
	//triangle->setNormalArray(Tnormal);
	triangle->setNormalBinding(osg::Geometry::BIND_OVERALL);

	/** set the Primitive use GL_TRIANGLES*/
	for (int i = 0; i < num - 2; i += 3)
	{
		osg::PrimitiveSet* Tprimitive = new osg::DrawArrays(GL_TRIANGLES, i, 3);
		triangle->addPrimitiveSet(Tprimitive);
	}

	return geode;
}

void Change(const vector<vector<cdt::Point*>>& pol, Area& pol1)
{
	vector<Point> polyline;
	for (int i = 0; i < pol.size(); i++)
	{
		for (int j = 0; j < pol[i].size(); j++)
		{
			polyline.push_back(Point(pol[i][j]->type, pol[i][j]->index, pol[i][j]->xyz));
		}
		if (i == 0)
		{
			pol1.addarea(polyline);
		}
		else
		{
			pol1.addhole(polyline);
		}
		polyline.clear();
	}
}

void SplitAreaToTriangle(const Area& thearea, vector<TriangleIndex>& splitresult)
{
	std::vector<cdt::Point*> polyline;
	std::vector<cdt::Point*> hole;
	cdt::CDT cdt = cdt::CDT();

	for (int i = 0; i < thearea.poly.size(); i++)
	{
		polyline.push_back(new cdt::Point(thearea.poly[i].type, thearea.poly[i].index, thearea.poly[i].xyz));
	}
	cdt.AddConstraint(polyline);

	for (int i = 0; i< thearea.hole.size(); i++)
	{
		for (int j = 0; j < thearea.hole[i].size(); j++)
		{
			hole.push_back(new cdt::Point(thearea.hole[i][j].type, thearea.hole[i][j].index, thearea.hole[i][j].xyz));
		}		
		cdt.AddHole(hole);
		hole.clear();
	}
	cdt.Triangulate();
	vector<cdt::Triangle*> triangles = cdt.GetTriangles();
	for (vector<cdt::Triangle*>::iterator it = triangles.begin(); it != triangles.end(); it++)
	{
		cdt::Triangle& t = **it;
		cdt::Point& a = *t.GetPoint(0);
		cdt::Point& b = *t.GetPoint(1);
		cdt::Point& c = *t.GetPoint(2);
		splitresult.push_back(TriangleIndex(PointIndex(a.type, a.index), PointIndex(b.type, b.index), PointIndex(c.type, c.index)));
	}
	FreeClear(polyline);
	FreeClear(hole);
}

void Writedata(string path1, const std::vector<cdt::Triangle*>& result)
{
	ofstream myfile(path1);
	for (int i = 0; i < result.size(); i++)
	{
		for (int j = 0; j < 3; j++)
			myfile << setiosflags(ios::fixed) << setprecision(5) << result[i]->GetPoint(j)->xyz.x() << " " << result[i]->GetPoint(j)->xyz.y() << " " << result[i]->GetPoint(j)->xyz.z() << endl;
	}
	myfile.close();
}

//可以进行改进，当flag为false时，不添加洞即可
void Triangulated(const vector<Point>& outline, const vector<Point>& points, const vector<vector<Point>>& holes, bool flag, vector<TriangleIndex>& splitresult)
{
	std::vector<cdt::Point*> polyline;
	std::vector<cdt::Point*> hole;
	std::vector<osg::Vec3d> polygon;
	osg::Vec3d centroid;
	cdt::CDT cdt = cdt::CDT();

	for (int i = 0; i < outline.size(); i++)
	{
		polyline.push_back(new cdt::Point(outline[i].type, outline[i].index, outline[i].xyz));
		polygon.push_back(outline[i].xyz);
	}
	cdt.AddConstraint(polyline);

	for (int i = 0; i < holes.size(); i++)
	{
		for (int j = 0; j < holes[i].size(); j++)
		{
			hole.push_back(new cdt::Point(holes[i][j].type, holes[i][j].index, holes[i][j].xyz));
		}
		cdt.AddHole(hole);
		hole.clear();
	}

	for (int i = 0; i < points.size(); i++)
	{
		cdt.AddPoint(new cdt::Point(points[i].type, points[i].index, points[i].xyz));
	}

	cdt.Triangulate();
	if (flag)
	{
		vector<cdt::Triangle*> triangles = cdt.GetTriangles();
		for (vector<cdt::Triangle*>::iterator it = triangles.begin(); it != triangles.end(); it++)
		{
			cdt::Triangle& t = **it;
			cdt::Point& a = *t.GetPoint(0);
			cdt::Point& b = *t.GetPoint(1);
			cdt::Point& c = *t.GetPoint(2);
			splitresult.push_back(TriangleIndex(PointIndex(a.type, a.index), PointIndex(b.type, b.index), PointIndex(c.type, c.index)));
		}
	}
	else
	{
		list<cdt::Triangle*> triangles = cdt.GetMap();
		for (list<cdt::Triangle*>::iterator it = triangles.begin(); it != triangles.end(); it++)
		{
			cdt::Triangle& t = **it;
			cdt::Point& a = *t.GetPoint(0);
			cdt::Point& b = *t.GetPoint(1);
			cdt::Point& c = *t.GetPoint(2);

			centroid = (a.xyz + b.xyz + c.xyz) / 3;
			
			if (PointinPoly(polygon, centroid))
			{
				splitresult.push_back(TriangleIndex(PointIndex(a.type, a.index), PointIndex(b.type, b.index), PointIndex(c.type, c.index)));
			}
		}
	}
	FreeClear(polyline);
	FreeClear(hole);
}

void Triangulated(const vector<Point>& outline, const vector<Point>& points, const vector<vector<Point>>& holes, bool flag, vector<Triangle>& splitresult)
{
	std::vector<cdt::Point*> polyline;
	std::vector<cdt::Point*> hole;
	std::vector<osg::Vec3d> polygon;
	osg::Vec3d centroid;
	cdt::CDT cdt = cdt::CDT();

	for (int i = 0; i < outline.size(); i++)
	{
		polyline.push_back(new cdt::Point(outline[i].type, outline[i].index, outline[i].xyz));
		polygon.push_back(outline[i].xyz);
	}
	cdt.AddConstraint(polyline);

	for (int i = 0; i < holes.size(); i++)
	{
		for (int j = 0; j < holes[i].size(); j++)
		{
			hole.push_back(new cdt::Point(holes[i][j].type, holes[i][j].index, holes[i][j].xyz));			
		}
		cdt.AddHole(hole);
		hole.clear();
	}

	for (int i = 0; i < points.size(); i++)
	{
		cdt.AddPoint(new cdt::Point(points[i].type, points[i].index, points[i].xyz));
	}

	cdt.Triangulate();
	if (flag)
	{
		vector<cdt::Triangle*> triangles = cdt.GetTriangles();
		for (vector<cdt::Triangle*>::iterator it = triangles.begin(); it != triangles.end(); it++)
		{
			cdt::Triangle& t = **it;
			cdt::Point& a = *t.GetPoint(0);
			cdt::Point& b = *t.GetPoint(1);
			cdt::Point& c = *t.GetPoint(2);
			splitresult.push_back(Triangle(Point(a.type, a.index, a.xyz), Point(b.type, b.index, b.xyz), Point(c.type, c.index, c.xyz)));
		}
	}
	else //目前存在问题，当多边形为凹时，所构成的三角网为凸集，可以判断三角形重心（x1+x2+x3/3,y1+y2+y3/3,z1+z2+z3/3）是否位于多边形内
	{
		list<cdt::Triangle*> triangles = cdt.GetMap();
		for (list<cdt::Triangle*>::iterator it = triangles.begin(); it != triangles.end(); it++)
		{
			cdt::Triangle& t = **it;
			cdt::Point& a = *t.GetPoint(0);
			cdt::Point& b = *t.GetPoint(1);
			cdt::Point& c = *t.GetPoint(2);
			
			centroid = (a.xyz + b.xyz + c.xyz) / 3;
			if (PointinPoly(polygon, centroid))
			{
				splitresult.push_back(Triangle(Point(a.type, a.index, a.xyz), Point(b.type, b.index, b.xyz), Point(c.type, c.index, c.xyz)));
			}
		}
	}
	FreeClear(polyline);
	FreeClear(hole);
}

void Triangulated(const vector<Point>& points, vector<TriangleIndex>& splitresult)
{
	std::vector<cdt::Point*> polyline;
	cdt::CDT cdt = cdt::CDT();

	double xmax(points[0].xyz.x()), xmin(points[0].xyz.x());
	double ymax(points[0].xyz.y()), ymin(points[0].xyz.y());

	for (unsigned int i = 0; i < points.size(); i++)
	{
		if (points[i].xyz.x() > xmax)
			xmax = points[i].xyz.x();
		if (points[i].xyz.x() < xmin)
			xmin = points[i].xyz.x();
		if (points[i].xyz.y() > ymax)
			ymax = points[i].xyz.y();
		if (points[i].xyz.y() < ymin)
			ymin = points[i].xyz.y();
	}

	double dx = 0.2 * (xmax - xmin);
	double dy = 0.2 * (ymax - ymin);

	cdt::Point* p1 = new cdt::Point(xmin - dx, ymin - dy);
	cdt::Point* p2 = new cdt::Point(xmax + dx, ymin - dy);
	cdt::Point* p3 = new cdt::Point(xmax + dx, ymax + dy);
	cdt::Point* p4 = new cdt::Point(xmin - dx, ymax + dy);

	polyline.push_back(p1);
	polyline.push_back(p2);
	polyline.push_back(p3);
	polyline.push_back(p4);
	cdt.AddConstraint(polyline);

	for (int i = 0; i < points.size(); i++)
	{
		cdt.AddPoint(new cdt::Point(points[i].type, points[i].index, points[i].xyz));
	}

	cdt.Triangulate();

	list<cdt::Triangle*> triangles = cdt.GetMap();
	for (list<cdt::Triangle*>::iterator it = triangles.begin(); it != triangles.end(); it++)
	{
		cdt::Triangle& t = **it;
		if (!(t.Contains(p1) || t.Contains(p2) || t.Contains(p3) || t.Contains(p4)))
		{
			cdt::Point& a = *t.GetPoint(0);
			cdt::Point& b = *t.GetPoint(1);
			cdt::Point& c = *t.GetPoint(2);
			splitresult.push_back(TriangleIndex(PointIndex(a.type, a.index), PointIndex(b.type, b.index), PointIndex(c.type, c.index)));
		}

	}
	FreeClear(polyline);
}

void Triangulated(const vector<Point>& points, vector<Triangle>& splitresult)
{
	std::vector<cdt::Point*> polyline;
	cdt::CDT cdt = cdt::CDT();

	double xmax(points[0].xyz.x()), xmin(points[0].xyz.x());
	double ymax(points[0].xyz.y()), ymin(points[0].xyz.y());

	for (unsigned int i = 0; i < points.size(); i++)
	{
		if (points[i].xyz.x() > xmax)
			xmax = points[i].xyz.x();
		if (points[i].xyz.x() < xmin)
			xmin = points[i].xyz.x();
		if (points[i].xyz.y() > ymax)
			ymax = points[i].xyz.y();
		if (points[i].xyz.y() < ymin)
			ymin = points[i].xyz.y();
	}

	double dx = 0.2 * (xmax - xmin);
	double dy = 0.2 * (ymax - ymin);
	cdt::Point* p1 = new cdt::Point(xmin - dx, ymin - dy);
	cdt::Point* p2 = new cdt::Point(xmax + dx, ymin - dy);
	cdt::Point* p3 = new cdt::Point(xmax + dx, ymax + dy);
	cdt::Point* p4 = new cdt::Point(xmin - dx, ymax + dy);

	polyline.push_back(p1);
	polyline.push_back(p2);
	polyline.push_back(p3);
	polyline.push_back(p4);
	cdt.AddConstraint(polyline);

	for (int i = 0; i < points.size(); i++)
	{
		cdt.AddPoint(new cdt::Point(points[i].type, points[i].index, points[i].xyz));
	}

	cdt.Triangulate();

	list<cdt::Triangle*> triangles = cdt.GetMap();
	for (list<cdt::Triangle*>::iterator it = triangles.begin(); it != triangles.end(); it++)
	{
		cdt::Triangle& t = **it;
		if (!(t.Contains(p1) || t.Contains(p2) || t.Contains(p3) || t.Contains(p4)))
		{
			cdt::Point& a = *t.GetPoint(0);
			cdt::Point& b = *t.GetPoint(1);
			cdt::Point& c = *t.GetPoint(2);
			splitresult.push_back(Triangle(Point(a.type, a.index, a.xyz), Point(b.type, b.index, b.xyz), Point(c.type, c.index, c.xyz)));
		}
	}
	FreeClear(polyline);
}

//判断点是否位于多边形内,在为true，否则false
bool PointinPoly(std::vector<osg::Vec3d>polygon, osg::Vec3d p)
{
	double max_x = polygon[0].x();
	double max_y = polygon[0].y();
	double min_x = polygon[0].x();
	double min_y = polygon[0].y();
	double temp;
	for (int i = 1; i < polygon.size(); i++)
	{
		if (min_x - polygon[i].x() > 0.0000001)
			min_x = (double)polygon[i].x();
		if (min_y - polygon[i].y() > 0.0000001)
			min_y = (double)polygon[i].y();
		if (max_x - polygon[i].x() < -0.0000001)
			max_x = (double)polygon[i].x();
		if (max_y - polygon[i].y() < -0.0000001)
			max_y = (double)polygon[i].y();
	}

	if (p.x() - min_x<-0.0000001 || p.x() - max_x>-0.0000001 || p.y() - min_y<-0.0000001 || p.y() - max_y>0.0000001)
		return false;

	int j = polygon.size() - 1;
	bool  crosNums = false;

	for (int i = 0; i<polygon.size(); i++)
	{
		double temp = polygon[i].x() + (p.y() - polygon[i].y()) / (polygon[j].y() - polygon[i].y())*(polygon[j].x() - polygon[i].x());
		if ((polygon[i].y()< p.y() && polygon[j].y() >= p.y() || polygon[j].y()<p.y() && polygon[i].y() >= p.y()) && (polygon[i].x() <= p.x() || polygon[j].x() <= p.x()))
		{
			crosNums ^= (polygon[i].x() + (p.y() - polygon[i].y()) / (polygon[j].y() - polygon[i].y())*(polygon[j].x() - polygon[i].x())<p.x());
		}

		////判断点是否位于多边形边上，如果测试点刚好在多边形的边上，则这种算法得到的结果是不确定的，因此以下算法判断点是否位于多边形边上，若点位于多边形边上则判定该点在多边形内
		if ((polygon[j].y() - polygon[i].y()) != 0 && abs((polygon[i].x() + (p.y() - polygon[i].y()) / (polygon[j].y() - polygon[i].y())*(polygon[j].x() - polygon[i].x()) - p.x())) < 0.0000001)
		{
			return true;
		}
		else if ((polygon[j].x() - polygon[i].x()) != 0 && abs((polygon[i].y() + (p.x() - polygon[i].x()) / (polygon[j].x() - polygon[i].x())*(polygon[j].y() - polygon[i].y()) - p.y())) < 0.0000001)
		{
			return true;
		}
		j = i;
	}
	return crosNums;
}