#include"geos.h"
#include"geos/geom/Coordinate.h"
#include"geos/geom/CoordinateArraySequence.h"
#include <geos/geom/Geometry.h>
#include <geos/algorithm/CGAlgorithms.h>
#include <geos/algorithm/LineIntersector.h>
#include <geos/triangulate/DelaunayTriangulationBuilder.h>
#include <geos/geom/GeometryCollection.h>

#include <osg/Array>
#include <osg/Vec3d>
#include <osg/ref_ptr>

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

using namespace std;
//using namespace osg;

const double EPS = 0.01;

void test();
void test1();
string str(bool flag);
double StringToDouble(const std::string& s);
void Readdata(string path1, osg::ref_ptr<osg::Vec3dArray> &polygon1);
void Writedata(string path1, vector<osg::ref_ptr<osg::Vec3dArray>> result);

string Trim(string& str)
{
	//str.find_first_not_of(" \t\r\n"),在字符串str中从索引0开始，返回首次不匹配"\t\r\n"的位置  
	str.erase(0, str.find_first_not_of(" \t\r\n"));
	str.erase(str.find_last_not_of(" \t\r\n") + 1);
	return str;
}

struct Point1
{
	int type;
	int index;
	int flag;//为适应带岛屿多边形的三角剖分
	osg::Vec3d xyz;
	Point1(){}
	~Point1(){}
	Point1(const osg::Vec3d& inxyz)
	{
		type = 0;
		index = 0;
		flag = 0;
		xyz = inxyz;
	}
	Point1(const int& intype, const osg::Vec3d& inxyz)
	{
		type = intype;
		xyz = inxyz;
	}
	Point1(const int& intype, const int& inindex, const osg::Vec3d& inxyz)
	{
		type = intype;
		index = inindex;
		xyz = inxyz;
	}
	bool operator==(const Point1& inp)
	{
		if (fabs(xyz[0] - inp.xyz[0]) <= 0.001&&fabs(xyz[1] - inp.xyz[1]) <= 0.001)
			return true;
		return false;
	}
	bool operator!=(const Point1& inp)
	{
		if (fabs(xyz[0] - inp.xyz[0]) >= 0.001&&fabs(xyz[1] - inp.xyz[1]) >= 0.001)
			return true;
		return false;
	}
	bool operator!=(const Point1& inp) const
	{
		if (fabs(xyz[0] - inp.xyz[0]) >= 0.001&&fabs(xyz[1] - inp.xyz[1]) >= 0.001)
			return true;
		return false;
	}
	bool operator==(const Point1& inp) const
	{
		if (fabs(xyz[0] - inp.xyz[0]) <= 0.001&&fabs(xyz[1] - inp.xyz[1]) <= 0.001)
			return true;
		return false;
	}
	void operator=(const Point1& inp)
	{
		xyz = inp.xyz;
		index = inp.index;
		type = inp.type;
		flag = inp.flag;
	}
};

struct Area
{
	vector<Point1> poly;
	vector<vector<Point1>> hole;
	Area(){}
	~Area(){}
	Area(vector<Point1>& inpoly)
	{
		poly.insert(poly.begin(), inpoly.begin(), inpoly.end());
	}
	void addhole(vector<Point1>& inhole)
	{
		hole.push_back(inhole);
	}
};

void Writedata(string path1, vector<Area> result);
bool PointinPolygon(const osg::ref_ptr<osg::Vec3dArray>& polygon1, osg::Vec3d point);
bool PolygonDifference(const osg::ref_ptr<osg::Vec3dArray>& polygon1, const osg::ref_ptr<osg::Vec3dArray>& polygon2, vector<osg::ref_ptr<osg::Vec3dArray>> &result);
bool PolygonDifference(const osg::ref_ptr<osg::Vec3dArray>& polygon1, const vector<osg::ref_ptr<osg::Vec3dArray>>& polygon2, vector<osg::ref_ptr<osg::Vec3dArray>> &result);
bool PolygonDifference1(const osg::ref_ptr<osg::Vec3dArray>& polygon1, const vector<osg::ref_ptr<osg::Vec3dArray>>& polygon2, vector<osg::ref_ptr<osg::Vec3dArray>> &result);
//bool PolDifference(vector<osg::ref_ptr<osg::Vec3dArray>> polygon,vector<osg::ref_ptr<osg::Vec3dArray>> &result);

bool PolygonIntersection(const osg::ref_ptr<osg::Vec3dArray>& polygon1, const osg::ref_ptr<osg::Vec3dArray>& polygon2, vector<osg::ref_ptr<osg::Vec3dArray>> &result);
bool PolygonIntersection(const osg::ref_ptr<osg::Vec3dArray>& polygon1, const vector<osg::ref_ptr<osg::Vec3dArray>>& polygon2, vector<osg::ref_ptr<osg::Vec3dArray>> &result);
bool PolygonIntersection1(const osg::ref_ptr<osg::Vec3dArray>& polygon1, const vector<osg::ref_ptr<osg::Vec3dArray>>& polygon2, vector<osg::ref_ptr<osg::Vec3dArray>> &result);
bool PolygonIntersection(const vector<osg::ref_ptr<osg::Vec3dArray>>& polygon, vector<osg::ref_ptr<osg::Vec3dArray>> &result);//求所有多边形的公共交集

bool PolygonUnion(const osg::ref_ptr<osg::Vec3dArray>& polygon1, const osg::ref_ptr<osg::Vec3dArray>& polygon2, vector<osg::ref_ptr<osg::Vec3dArray>> &result);
bool PolygonUnion(const osg::ref_ptr<osg::Vec3dArray>& polygon1, const osg::ref_ptr<osg::Vec3dArray>& polygon2, vector<Area> &result);
bool PolygonUnion(const osg::ref_ptr<osg::Vec3dArray>& polygon1, const vector<osg::ref_ptr<osg::Vec3dArray>>& polygon2, vector<osg::ref_ptr<osg::Vec3dArray>> &result);
bool PolygonUnion(const vector<osg::ref_ptr<osg::Vec3dArray>>& polygon, vector<Area> &result);
//geos::geom::Geometry* PolygonUnion(const vector<osg::ref_ptr<osg::Vec3dArray>>& polygon,geos::geom::Geometry* a);

bool PolygonsymDifference(const osg::ref_ptr<osg::Vec3dArray>& polygon1,const osg::ref_ptr<osg::Vec3dArray>& polygon2, vector<osg::ref_ptr<osg::Vec3dArray>> &result);
bool PolygonsymDifference(const osg::ref_ptr<osg::Vec3dArray>& polygon1, const vector<osg::ref_ptr<osg::Vec3dArray>>& polygon2, vector<Area> &result);


int main(int argc, char *argv[])
{
	/*osg::ref_ptr<osg::Vec3dArray> polygon1 = new osg::Vec3dArray();
	polygon1->push_back(osg::Vec3d(0, 0, 1));
	polygon1->push_back(osg::Vec3d(3, 0, 2));
	polygon1->push_back(osg::Vec3d(3, 3, 3));
	polygon1->push_back(osg::Vec3d(0, 3, 1));

	osg::ref_ptr<osg::Vec3dArray> polygon2 = new osg::Vec3dArray();
	polygon2->push_back(osg::Vec3d(-1, 1, 1));
	polygon2->push_back(osg::Vec3d(4, 1, 2));
	polygon2->push_back(osg::Vec3d(4, 2, 3));
	polygon2->push_back(osg::Vec3d(-1, 2, 4));

	osg::ref_ptr<osg::Vec3dArray> polygon3 = new osg::Vec3dArray();
	polygon3->push_back(osg::Vec3d(1, -1, 1));
	polygon3->push_back(osg::Vec3d(2, -1, 2));
	polygon3->push_back(osg::Vec3d(2, 4, 3));
	polygon3->push_back(osg::Vec3d(1, 4, 4));*/

	//osg::ref_ptr<osg::Vec3dArray> polygon1 = new osg::Vec3dArray();
	//polygon1->push_back(osg::Vec3d(0, 0, 1));
	//polygon1->push_back(osg::Vec3d(3, 0, 2));
	//polygon1->push_back(osg::Vec3d(3, 3, 3));
	//polygon1->push_back(osg::Vec3d(0, 3, 4));
	//polygon1->push_back(osg::Vec3d(0, 0, 1));

	//osg::ref_ptr<osg::Vec3dArray> polygon2 = new osg::Vec3dArray();
	//polygon2->push_back(osg::Vec3d(1, 1, 0));
	//polygon2->push_back(osg::Vec3d(2, 1, 0));
	//polygon2->push_back(osg::Vec3d(2, 2, 0));
	//polygon2->push_back(osg::Vec3d(1, 2, 0));
	//polygon2->push_back(osg::Vec3d(1, 1, 0));

	//osg::ref_ptr<osg::Vec3dArray> polygon3 = new osg::Vec3dArray();
	//polygon3->push_back(osg::Vec3d(2, -1, 0));
	//polygon3->push_back(osg::Vec3d(4, -1, 0));
	//polygon3->push_back(osg::Vec3d(4, 1, 0));
	//polygon3->push_back(osg::Vec3d(2, 1, 0));
	//polygon3->push_back(osg::Vec3d(2, -1, 0));
	///*polygon3->push_back(osg::Vec3d(6, 0, 1));
	//polygon3->push_back(osg::Vec3d(8, 0, 2));
	//polygon3->push_back(osg::Vec3d(8, 3, 3));
	//polygon3->push_back(osg::Vec3d(6, 3, 4));*/

	//vector<osg::ref_ptr<osg::Vec3dArray>> polygon4;
	//vector<osg::ref_ptr<osg::Vec3dArray>> polygon5;
	//polygon5.push_back(polygon1);
	//polygon4.push_back(polygon2);
	//polygon4.push_back(polygon3);
	////polygon4.push_back(polygon1);


	//vector<osg::ref_ptr<osg::Vec3dArray>> result1;
	//vector<osg::ref_ptr<osg::Vec3dArray>> result2;
	//vector<osg::ref_ptr<osg::Vec3dArray>> result3;
	//vector<osg::ref_ptr<osg::Vec3dArray>> result4;

	//cout << "Difference:" << endl;
	//bool flag1 = PolygonDifference(polygon1,polygon3,result1);
	//cout << "Intersection:" << endl;
	//bool flag2 = PolygonIntersection(polygon1, polygon3, result2);
	//cout << "Union:" << endl;
	//bool flag3 = PolygonUnion(polygon1,polygon3, result3);
	//cout << "symDifference:" << endl;
	//bool flag4 = PolygonsymDifference(polygon1, polygon3, result4);
	//for (int i = 0; i < result3.size(); i++)
	//{
	//	cout << i+1<<":"<< endl;
	//	for (int j = 0; j < result3[i]->size(); j++)
	//	{
	//		cout << result3[i]->at(j).x() << " " << result3[i]->at(j).y() << " " << result3[i]->at(j).z() << endl;
	//	}
	//}

	string path1 = "E://练习//PolygonOverlay//PolygonOverlay//data//求交//test2//mask.txt";
	string path2 = "E://练习//PolygonOverlay//PolygonOverlay//data//求交//test2//outline.txt";
	/*string path1 = "E://练习//PolygonOverlay//PolygonOverlay//data//求交//1.txt";
	string path2 = "E://练习//PolygonOverlay//PolygonOverlay//data//求交//2.txt";
	string path3 = "E://练习//PolygonOverlay//PolygonOverlay//data//求交//3.txt";
	string path4 = "E://练习//PolygonOverlay//PolygonOverlay//data//求交//4.txt";
	string path5 = "E://练习//PolygonOverlay//PolygonOverlay//data//求交//5.txt";
	string path6 = "E://练习//PolygonOverlay//PolygonOverlay//data//求交//6.txt";
	string path7 = "E://练习//PolygonOverlay//PolygonOverlay//data//求交//7.txt";
	string path8 = "E://练习//PolygonOverlay//PolygonOverlay//data//求交//8.txt";
	string path9 = "E://练习//PolygonOverlay//PolygonOverlay//data//求交//9.txt";
	string path10 = "E://练习//PolygonOverlay//PolygonOverlay//data//求交//10.txt";*/
	string outpath = "E://练习//PolygonOverlay//PolygonOverlay//data//求交//test2//result6.txt";
	osg::ref_ptr<osg::Vec3dArray> polygon1 = new osg::Vec3dArray();
	osg::ref_ptr<osg::Vec3dArray> polygon2 = new osg::Vec3dArray();
	/*osg::ref_ptr<osg::Vec3dArray> polygon3 = new osg::Vec3dArray();
	osg::ref_ptr<osg::Vec3dArray> polygon4 = new osg::Vec3dArray();
	osg::ref_ptr<osg::Vec3dArray> polygon5 = new osg::Vec3dArray();
	osg::ref_ptr<osg::Vec3dArray> polygon6 = new osg::Vec3dArray();
	osg::ref_ptr<osg::Vec3dArray> polygon7 = new osg::Vec3dArray();
	osg::ref_ptr<osg::Vec3dArray> polygon8 = new osg::Vec3dArray();
	osg::ref_ptr<osg::Vec3dArray> polygon9 = new osg::Vec3dArray();
	osg::ref_ptr<osg::Vec3dArray> polygon10 = new osg::Vec3dArray();*/
	vector<osg::ref_ptr<osg::Vec3dArray>> polygon;

	Readdata(path1, polygon1);
	Readdata(path2, polygon2);
	/*Readdata(path3, polygon3);
	Readdata(path4, polygon4);
	Readdata(path5, polygon5);
	Readdata(path6, polygon6);
	Readdata(path7, polygon7);
	Readdata(path8, polygon8);
	Readdata(path9, polygon9);
	Readdata(path10, polygon10);*/

	polygon.push_back(polygon1);
	/*polygon.push_back(polygon2);
	polygon.push_back(polygon3);
	polygon.push_back(polygon4);
	polygon.push_back(polygon5);
	polygon.push_back(polygon6);
	polygon.push_back(polygon7);
	polygon.push_back(polygon8);
	polygon.push_back(polygon9);
	polygon.push_back(polygon10);*/

	vector<osg::ref_ptr<osg::Vec3dArray>> result1,result4;
	vector<Area> result, result5;
	bool flag1 = PolygonIntersection(polygon1, polygon2, result1);
	//bool flag2 = PolygonDifference(polygon1,polygon, result1);
	//bool flag3 = PolygonUnion(polygon, result);
	//bool flag4 = PolygonsymDifference(polygon1, polygon, result5);
	//bool flag3 = PolygonIntersection(polygon1, polygon2, result1);
	Writedata(outpath,result1);

//点是否位于多边形内
 //   string path1 = "E://练习//PolygonOverlay//PolygonOverlay//data//mask3.txt";
	//osg::ref_ptr<osg::Vec3dArray> polygon1 = new osg::Vec3dArray();
	//Readdata(path1, polygon1);
	////osg::Vec3d point = { 5248.0000, 27314.4108, 0.0000}; //外部
	////osg::Vec3d point = { 5295.418,27334.196,1984.801 }; //内部
	//osg::Vec3d point = { 5295.418, 27334.196, 1984.801 }; //边上
	//bool flag = PointinPolygon(polygon1, point);

	//typedef Coordinate PT;
	//const PrecisionModel* p=new PrecisionModel(10);
	//int a = p->getMaximumSignificantDigits();
	//geos::geom::GeometryFactory* factory=new GeometryFactory(p);
	//const PrecisionModel* p12=factory->getPrecisionModel();
	//double re=p->makePrecise(0.151);// 把0.151设置为指定的PrecisionModel精度
	//Point* p1 = factory->createPoint(PT(0.15, 0, 1));
	//Point* p2 = factory->createPoint(PT(0.151, 0, 5));
	//bool presult = p1->equalsExact(p2);
	//delete p;
	//delete factory;
	//int num = presult->getNumGeometries();
	system("pause");
	return 1;
}


void test()
{
	cout << "GEOS库版本为：" << GEOS_VERSION << endl;

	typedef Coordinate PT;
	geos::geom::GeometryFactory factory;
	
	geos::geom::CoordinateArraySequenceFactory csf; //构建第一个矩形p1
	CoordinateSequence* cs1 = csf.create(5, 3);//五个2维点，第三维度z始终为0
	cs1->setAt(PT(0, 0, 1), 0);
	cs1->setAt(PT(3, 0, 2), 1);
	cs1->setAt(PT(3, 3, 3), 2);
	cs1->setAt(PT(0, 3, 4), 3);
	cs1->setAt(PT(0, 0, 1), 4); //与第一个点相等，构成闭合
	LinearRing* ring1 = factory.createLinearRing(cs1); //点构成线
	Geometry* p1 = factory.createPolygon(ring1, NULL); //线构成面

	CoordinateSequence* cs2 = csf.create(5, 3); //构建一个四边形p2
	cs2->setAt(PT(4, 4, 5), 0);
	cs2->setAt(PT(4, 5, 6), 1);
	cs2->setAt(PT(5, 5, 7), 2);
	cs2->setAt(PT(5, 4, 8), 3);
	cs2->setAt(PT(4, 4, 5), 4);
	LinearRing * ring2 = factory.createLinearRing(cs2);
	Geometry* p2 = (factory.createPolygon(ring2, NULL));

#pragma region
	geos::geom::CoordinateSequence* ls1 = csf.create(2, 3);
	ls1->setAt(PT(3, 0, 2), 0);
	ls1->setAt(PT(3, 3, 3), 1);
	geos::geom::CoordinateSequence* ls2 = csf.create(2, 3);
	ls2->setAt(PT(2, 2, 0), 0);
	ls2->setAt(PT(2, 0, 8), 1);
	geos::geom::LineString *l1 = factory.createLineString(ls1);
	geos::geom::LineString *l2 = factory.createLineString(ls2);
	

	geos::geom::Geometry* intpoint = l1->intersection(l2);
	
	const geos::geom::CoordinateSequence *b = intpoint->getCoordinates();
	geos::geom::Coordinate p11 = PT(500243.1445, 4538669.141, 20);
	geos::geom::Coordinate p21 = PT(500273.1445, 4538699.141, 10);
	geos::geom::Coordinate q11 = PT(500250, 4538554, 0);
	geos::geom::Coordinate q21 = PT(500250, 4538676, 0);

	int Pq1 = geos::algorithm::CGAlgorithms::orientationIndex(p11, p21, q11);
	int Pq2 = geos::algorithm::CGAlgorithms::orientationIndex(p11, p21, q21);

	int Qp1 = geos::algorithm::CGAlgorithms::orientationIndex(q11, q21, p11);
	int Qp2 = geos::algorithm::CGAlgorithms::orientationIndex(q11, q21, p21);
	
	for (int i = 0; i < b->getSize(); i++)
	{
		cout << std::setprecision(13) << b->getAt(i).x << "  " << b->getAt(i).y << " " << b->getAt(i).z << endl;
	}

	//geos::geom::CoordinateSequence* cs1 = csf.create(4, 3);//五个2维点，第三维度z始终为0
	//cs1->setAt(PT(500243.1445, 4538699.141, 1), 0);
	//cs1->setAt(PT(500213.1445, 4538669.141, 1), 1);
	//cs1->setAt(PT(500243.1445, 4538669.141, 1), 2);
	//cs1->setAt(PT(500243.1445, 4538699.141, 1), 3);

	/*cs1->setAt(PT(500273.1445, 4538669.141, 1), 0);
	cs1->setAt(PT(500243.1445, 4538639.141, 1), 1);
	cs1->setAt(PT(500273.1445, 4538639.141, 1), 2);
	cs1->setAt(PT(500273.1445, 4538669.141, 1), 3);*/

	//geos::geom::LinearRing* ring1 = factory.createLinearRing(cs1); //点构成线
	//geos::geom::Geometry* p1 = factory.createPolygon(ring1, NULL); //线构成面

	//geos::geom::CoordinateSequence* cs2 = csf.create(5, 3); //构建一个四边形p2
	//cs2->setAt(PT(500130, 4538676, 100), 0);
	//cs2->setAt(PT(500250, 4538676, 100), 1);
	//cs2->setAt(PT(500250, 4538554, 100), 2);
	//cs2->setAt(PT(500130, 4538554, 100), 3);
	//cs2->setAt(PT(500130, 4538676, 100), 4);
	//geos::geom::LinearRing * ring2 = factory.createLinearRing(cs2);
	//geos::geom::Geometry* p2 = (factory.createPolygon(ring2, NULL));
#pragma endregion
	
	geos::geom::CoordinateSequence *cs3 = new CoordinateArraySequence(); //构建一个三角形p3
	int xoffset = 4, yoffset = 4, side = 2;
	cs3->add(PT(xoffset, yoffset));
	cs3->add(PT(xoffset, yoffset + side));
	cs3->add(PT(xoffset + side, yoffset + side));
	cs3->add(PT(xoffset, yoffset));
	geos::geom::LinearRing * ring3 = factory.createLinearRing(cs3);
	geos::geom::Geometry* p3 = (factory.createPolygon(ring3, NULL));

	bool flag12 = p1->intersects(p2);
	bool flag13 = p1->intersects(p3);
	bool flag23 = p2->intersects(p3);
	geos::geom::Geometry* p4 = p1->difference(p2);
	const geos::geom::Geometry* p5 = p4->getGeometryN(1);
	int n = p5->getNumGeometries();
	bool flag = p4->isSimple();
	int num = p4->getNumPoints();

	//三角化
	const geos::geom::CoordinateSequence *a = p4->getCoordinates();
	geos::triangulate::DelaunayTriangulationBuilder a1;
	a1.setSites(*p1);
	std::auto_ptr< geos::geom::GeometryCollection >aa=a1.getTriangles(factory);
	const geos::geom::CoordinateSequence *aa1 = aa->getCoordinates();
	for (int i = 0; i < aa1->getSize(); i++)
	{
		cout << std::setprecision(13) << aa1->getAt(i).x << "  " << aa1->getAt(i).y << " " << aa1->getAt(i).z << endl;
	}

	for (int i = 0; i < a->getSize(); i++)
	{
		cout << std::setprecision(13) << a->getAt(i).x << "  " << a->getAt(i).y << " " << a->getAt(i).z << endl;
	}
	cout << "图1与图2:" << str(flag12) << endl;
	cout << "图1与图3:" << str(flag13) << endl;
	cout << "图2与图3:" << str(flag23) << endl;
}

void test1()
{
	cout << "GEOS库版本为：" << GEOS_VERSION << endl;

	typedef Coordinate PT;
	geos::geom::GeometryFactory factory;

	geos::geom::CoordinateArraySequenceFactory csf; //构建第一个矩形p1
	CoordinateSequence* cs1 = csf.create(5, 3);//五个2维点，第三维度z始终为0
	cs1->setAt(PT(0, 0, 1), 0);
	cs1->setAt(PT(3, 0, 2), 1);
	cs1->setAt(PT(3, 3, 3), 2);
	cs1->setAt(PT(0, 3, 4), 3);
	cs1->setAt(PT(0, 0, 1), 4); //与第一个点相等，构成闭合
	LinearRing* ring1 = factory.createLinearRing(cs1); //点构成线
	Geometry* p1 = factory.createPolygon(ring1, NULL); //线构成面

	CoordinateSequence* cs2 = csf.create(5, 3); //构建一个四边形p2
	cs2->setAt(PT(-1, 1, 5), 0);
	cs2->setAt(PT(4, 1, 6), 1);
	cs2->setAt(PT(4, 2, 7), 2);
	cs2->setAt(PT(-1, 2, 8), 3);
	cs2->setAt(PT(-1, 1, 5), 4);
	LinearRing * ring2 = factory.createLinearRing(cs2);
	Geometry* p2 = (factory.createPolygon(ring2, NULL));

	CoordinateSequence* cs3 = csf.create(5, 3); //构建一个四边形p2
	cs3->setAt(PT(1, -1, 5), 0);
	cs3->setAt(PT(2, -1, 6), 1);
	cs3->setAt(PT(2, 4, 7), 2);
	cs3->setAt(PT(1, 4, 8), 3);
	cs3->setAt(PT(1, -1, 5), 4);
	LinearRing * ring3 = factory.createLinearRing(cs3);
	Geometry* p3 = (factory.createPolygon(ring3, NULL));

	geos::geom::Geometry* p4 = p1->difference(p2);
	geos::geom::Geometry* p5 = p4->difference(p3);
	p1 = p1->difference(p2);
	p1 = p1->difference(p3);
	
	int n = p1->getNumGeometries();
	for (int j = 0; j < n; j++)
	{
		const geos::geom::Geometry* p6 = p1->getGeometryN(j);
		const geos::geom::CoordinateSequence *b = p6->getCoordinates();

		for (int i = 0; i < b->getSize(); i++)
		{
			cout << std::setprecision(13) << b->getAt(i).x << "  " << b->getAt(i).y << " " << b->getAt(i).z << endl;
		}
		cout << endl;
	}
}

string str(bool flag)
{
	string result = (flag == true) ? "相交" : "不相交";
	return result;
}

//多边形求差集，结果为polygon1-polygon1与polygon2的交集 A-A∩B，其中若polygon2 z值至少一个为0，交点z值由polygon1插值得到；若polygon2 z值均不为0时，交点z值由polygon2插值得到
bool PolygonDifference(const osg::ref_ptr<osg::Vec3dArray>& polygon1, const osg::ref_ptr<osg::Vec3dArray>& polygon2, vector<osg::ref_ptr<osg::Vec3dArray>> &result)
{
	typedef Coordinate PT;
	geos::geom::GeometryFactory factory;
	geos::geom::CoordinateArraySequenceFactory csf;
	int num1 = polygon1->size();
	if (num1 < 3)
		return false;

	geos::geom::CoordinateSequence* cs1 = csf.create(num1 + 1, 3);
	for (int i = 0; i < num1; i++)
	{
		cs1->setAt(Coordinate(polygon1->at(i).x(), polygon1->at(i).y(), polygon1->at(i).z()), i);
		//std::cout << polygon1->at(i).x() << " " << polygon1->at(i).y() << " "<<polygon1->at(i).z() << endl;
	}
	cs1->setAt(Coordinate(polygon1->at(0).x(), polygon1->at(0).y(), polygon1->at(0).z()), num1);
	geos::geom::LinearRing* ring1 = factory.createLinearRing(cs1); //点构成线
	geos::geom:: Geometry* pol1 = factory.createPolygon(ring1, NULL);
	
	int num2 = polygon2->size();
	if (num2 < 3)
		return false;

	geos::geom::CoordinateSequence* cs2 = csf.create(num2 + 1, 3);
	for (int i = 0; i < num2; i++)
	{
		cs2->setAt(Coordinate(polygon2->at(i).x(), polygon2->at(i).y(), polygon2->at(i).z()), i);
	}
	cs2->setAt(Coordinate(polygon2->at(0).x(), polygon2->at(0).y(), polygon2->at(0).z()), num2);
	geos::geom::LinearRing* ring2 = factory.createLinearRing(cs2); //点构成线
	geos::geom::Geometry* pol2 = factory.createPolygon(ring2, NULL);

	geos::geom::Geometry* pol3 = pol1->difference(pol2);
	//bool falg = pol1->intersects(pol2);
	int geonum = pol3->getNumGeometries();

	for (int j = 0; j < geonum; j++)
	{
		osg::ref_ptr<osg::Vec3dArray> point1=new osg::Vec3dArray();
		const geos::geom::Geometry* pol4 = pol3->getGeometryN(j);
		const geos::geom::CoordinateSequence *a = pol4->getCoordinates();
		/*if ((pol1->intersects(pol2)))*/
		{
			for (int i = a->getSize() - 1; i > 0; i--)
			{
				point1->push_back(osg::Vec3d(a->getAt(i).x, a->getAt(i).y, a->getAt(i).z));
				cout << a->getAt(i).x << " " << a->getAt(i).y << " " << a->getAt(i).z << endl;
			}
		}
		/*else
		{
			for (int i = 0; i < a->getSize() - 1; i++)
			{
				point1->push_back(osg::Vec3d(a->getAt(i).x, a->getAt(i).y, a->getAt(i).z));
				cout << a->getAt(i).x << " " << a->getAt(i).y << " " << a->getAt(i).z << endl;
			}
		}*/
		
		cout << endl;
		result.push_back(point1);		
	}
	return true;
}

//一个多边形和多个多边形求差 A-（A∪B∪C∪...）
bool PolygonDifference(const osg::ref_ptr<osg::Vec3dArray>& polygon1, const vector<osg::ref_ptr<osg::Vec3dArray>>& polygon2, vector<osg::ref_ptr<osg::Vec3dArray>> &result)
{
	typedef Coordinate PT;
	geos::geom::GeometryFactory factory;
	geos::geom::CoordinateArraySequenceFactory csf;
	int num1 = polygon1->size();
	if (num1 < 3)
		return false;

	geos::geom::CoordinateSequence* cs1 = csf.create(num1 + 1, 3);
	for (int i = 0; i < num1; i++)
	{
		cs1->setAt(Coordinate(polygon1->at(i).x(), polygon1->at(i).y(), polygon1->at(i).z()), i);
	}
	cs1->setAt(Coordinate(polygon1->at(0).x(), polygon1->at(0).y(), polygon1->at(0).z()), num1);
	geos::geom::LinearRing* ring1 = factory.createLinearRing(cs1); //点构成线
	geos::geom::Geometry* pol1 = factory.createPolygon(ring1, NULL);
	geos::geom::Geometry* pol3 = pol1->clone();
	
	int polygonnum = polygon2.size();
	if (polygonnum < 1)
		return false;

	for (int i = 0; i < polygonnum; i++)
	{
		int num2 = polygon2[i]->size();
		if (num2 < 3)
			continue;

		geos::geom::CoordinateSequence* cs2 = csf.create(num2 + 1, 3);
		for (int j = 0; j < num2; j++)
		{
			cs2->setAt(Coordinate(polygon2[i]->at(j).x(), polygon2[i]->at(j).y(), polygon2[i]->at(j).z()), j);
		}
		cs2->setAt(Coordinate(polygon2[i]->at(0).x(), polygon2[i]->at(0).y(), polygon2[i]->at(0).z()), num2);
		geos::geom::LinearRing* ring2 = factory.createLinearRing(cs2); //点构成线
		geos::geom::Geometry* pol2 = factory.createPolygon(ring2, NULL);

		pol3 = pol3->difference(pol2);		
	}

	int geonum = pol3->getNumGeometries();
	if (geonum==0)
		return false;

	for (int j = 0; j < geonum; j++)
	{
		osg::ref_ptr<osg::Vec3dArray> point1 = new osg::Vec3dArray();
		const geos::geom::Geometry* pol4 = pol3->getGeometryN(j);
		const geos::geom::CoordinateSequence *a = pol4->getCoordinates();
		int temp = a->getSize();
		for (int i = a->getSize()-1; i >0; i--)
		{
			point1->push_back(osg::Vec3d(a->getAt(i).x, a->getAt(i).y, a->getAt(i).z));
			cout << a->getAt(i).x << " " << a->getAt(i).y << " " << a->getAt(i).z << endl;
		}
		cout << endl;
		result.push_back(point1);
	}
	return true;
}

//计算polygon1与vector中的多边形两两求差结果 A-A∩B、A-A∩C...
bool PolygonDifference1(const osg::ref_ptr<osg::Vec3dArray>& polygon1, const vector<osg::ref_ptr<osg::Vec3dArray>>& polygon2, vector<osg::ref_ptr<osg::Vec3dArray>> &result)
{
	geos::geom::GeometryFactory factory;
	geos::geom::CoordinateArraySequenceFactory csf;
	int num1 = polygon1->size();
	if (num1 < 3)
		return false;

	geos::geom::CoordinateSequence* cs1 = csf.create(num1 + 1, 3);
	for (int i = 0; i < num1; i++)
	{
		cs1->setAt(Coordinate(polygon1->at(i).x(), polygon1->at(i).y(), polygon1->at(i).z()), i);
	}
	cs1->setAt(Coordinate(polygon1->at(0).x(), polygon1->at(0).y(), polygon1->at(0).z()), num1);
	geos::geom::LinearRing* ring1 = factory.createLinearRing(cs1); //点构成线
	geos::geom::Geometry* pol1 = factory.createPolygon(ring1, NULL);

	int polygonnum = polygon2.size();
	if (polygonnum < 1)
		return false;

	geos::geom::Geometry* pol3;
	for (int i = 0; i < polygonnum; i++)
	{
		int num2 = polygon2[i]->size();
		if (num2 < 3)
			continue;

		geos::geom::CoordinateSequence* cs2 = csf.create(num2 + 1, 3);
		for (int j = 0; j < num2; j++)
		{
			cs2->setAt(Coordinate(polygon2[i]->at(j).x(), polygon2[i]->at(j).y(), polygon2[i]->at(j).z()), j);
		}
		cs2->setAt(Coordinate(polygon2[i]->at(0).x(), polygon2[i]->at(0).y(), polygon2[i]->at(0).z()), num2);
		geos::geom::LinearRing* ring2 = factory.createLinearRing(cs2); //点构成线
		geos::geom::Geometry* pol2 = factory.createPolygon(ring2, NULL);

		pol3 = pol1->difference(pol2);
		int geonum = pol3->getNumGeometries();
		for (int j = 0; j < geonum; j++)
		{
			osg::ref_ptr<osg::Vec3dArray> point1 = new osg::Vec3dArray();
			const geos::geom::Geometry* pol4 = pol3->getGeometryN(j);
			const geos::geom::CoordinateSequence *a = pol4->getCoordinates();
			int temp = a->getSize();
			for (int i = a->getSize() - 1; i > 0; i--)
			{
				point1->push_back(osg::Vec3d(a->getAt(i).x, a->getAt(i).y, a->getAt(i).z));
				cout << a->getAt(i).x << " " << a->getAt(i).y << " " << a->getAt(i).z << endl;
			}
			cout << endl;
			result.push_back(point1);
		}
	}
	if (result.size()>0)
		return true;
	else
		return false;
}

//输入多个多边形，计算两两求差结果
//bool PolDifference(const vector<osg::ref_ptr<osg::Vec3dArray>>& polygon, vector<osg::ref_ptr<osg::Vec3dArray>> &result)
//{
//	typedef Coordinate PT;
//	geos::geom::GeometryFactory factory;
//	geos::geom::CoordinateArraySequenceFactory csf;
//	int num1 = polygon.size();
//	
//	for (int i = 0; i < num1; i++)
//	{
//		int num2 = polygon[i]->size();
//		geos::geom::CoordinateSequence* cs1 = csf.create(num2 + 1, 3);
//		for (int j = 0; j < num2; j++)
//		{
//			cs1->setAt(Coordinate(polygon[i]->at(j).x(), polygon[i]->at(j).y(), polygon[i]->at(j).z()), j);
//		}
//		cs1->setAt(Coordinate(polygon[i]->at(0).x(), polygon[i]->at(0).y(), polygon[i]->at(0).z()), num2);
//		geos::geom::LinearRing* ring1 = factory.createLinearRing(cs1); //点构成线
//		geos::geom::Geometry* pol1 = factory.createPolygon(ring1, NULL);
//
//		for (int ii = 0; ii < num1; ii++)
//		{
//			if (ii == i)
//				continue;
//
//			int num3 = polygon[ii]->size();
//			geos::geom::CoordinateSequence* cs2 = csf.create(num3 + 1, 3);
//			for (int jj = 0; jj < num3; jj++)
//			{
//				cs2->setAt(Coordinate(polygon[ii]->at(jj).x(), polygon[ii]->at(jj).y(), polygon[ii]->at(jj).z()), jj);
//			}
//			cs2->setAt(Coordinate(polygon[ii]->at(0).x(), polygon[ii]->at(0).y(), polygon[ii]->at(0).z()), num3);
//			geos::geom::LinearRing* ring2 = factory.createLinearRing(cs2); //点构成线
//			geos::geom::Geometry* pol2 = factory.createPolygon(ring2, NULL);
//
//			if (pol1->intersects(pol2) == false)
//			{
//				continue;
//			}
//
//			geos::geom::Geometry* pol3 = pol1->difference(pol2);
//
//			int geonum = pol3->getNumGeometries();
//
//			for (int n = 0; n < geonum-1; n++)
//			{
//				osg::ref_ptr<osg::Vec3dArray> point1 = new osg::Vec3dArray();
//				const geos::geom::Geometry* pol4 = pol3->getGeometryN(n);
//				const geos::geom::CoordinateSequence *a = pol4->getCoordinates();
//				int temp = a->getSize();
//				for (int nn = 0; nn < a->getSize()-1; nn++)
//				{
//					point1->push_back(osg::Vec3d(a->getAt(nn).x, a->getAt(nn).y, a->getAt(nn).z));
//					cout << a->getAt(nn).x << " " << a->getAt(nn).y << " " << a->getAt(nn).z << endl;
//				}
//				cout << endl;
//				result.push_back(point1);
//			}
//		}
//	}
//	if (result.size() != 0)
//		return true;
//	else
//		return false;
//}

//计算多边形交集 A∩B
bool PolygonIntersection(const osg::ref_ptr<osg::Vec3dArray>& polygon1, const osg::ref_ptr<osg::Vec3dArray>& polygon2, vector<osg::ref_ptr<osg::Vec3dArray>> &result)
{
	typedef Coordinate PT;
	geos::geom::GeometryFactory factory;
	geos::geom::CoordinateArraySequenceFactory csf;
	int num1 = polygon1->size();
	if (num1 < 3)
		return false;

	geos::geom::CoordinateSequence* cs1 = csf.create(num1, 3);
	for (int i = 0; i < num1; i++)
	{
		double aa = floor(polygon1->at(i).y()*100) / 100;
		cs1->setAt(Coordinate(polygon1->at(i).x(), polygon1->at(i).y(), polygon1->at(i).z()), i);
		//std::cout << polygon1->at(i).x() << " " << polygon1->at(i).y() << " "<<polygon1->at(i).z() << endl;
	}
	//cs1->setAt(Coordinate(polygon1->at(0).x(), polygon1->at(0).y(), polygon1->at(0).z()), num1);
	geos::geom::LinearRing* ring1 = factory.createLinearRing(cs1); //点构成线
	geos::geom::Geometry* pol1 = factory.createPolygon(ring1, NULL);

	int num2 = polygon2->size();
	if (num2 < 3)
		return false;

	geos::geom::CoordinateSequence* cs2 = csf.create(num2, 3);
	for (int i = 0; i < num2; i++)
	{
		cs2->setAt(Coordinate(polygon2->at(i).x(), polygon2->at(i).y(), polygon2->at(i).z()), i);
	}
	//cs2->setAt(Coordinate(polygon2->at(0).x(), polygon2->at(0).y(), polygon2->at(0).z()), num2);
	geos::geom::LinearRing* ring2 = factory.createLinearRing(cs2); //点构成线
	geos::geom::Geometry* pol2 = factory.createPolygon(ring2, NULL);

	if (!(pol1->intersects(pol2)))
	{
		return false;
	}

	geos::geom::Geometry* pol3 = pol1->intersection(pol2);
	int geonum = pol3->getNumGeometries();

	for (int j = 0; j < geonum; j++)
	{
		osg::ref_ptr<osg::Vec3dArray> point1 = new osg::Vec3dArray();
		const geos::geom::Geometry* pol4 = pol3->getGeometryN(j);
		const geos::geom::CoordinateSequence *a = pol4->getCoordinates();
		int temp = a->getSize();
		for (int i = a->getSize() - 1; i >=0; i--)
		{
			point1->push_back(osg::Vec3d(a->getAt(i).x, a->getAt(i).y, a->getAt(i).z));
			cout <<setiosflags(ios::fixed) << setprecision(5) << a->getAt(i).x << " " << a->getAt(i).y << " " << a->getAt(i).z << endl;
		}
		cout << endl;
		result.push_back(point1);
	}
	return true;
}

//求指定多边形和多个多边形的交集，A∩B∩C...
bool PolygonIntersection(const osg::ref_ptr<osg::Vec3dArray>& polygon1, const vector<osg::ref_ptr<osg::Vec3dArray>>& polygon2, vector<osg::ref_ptr<osg::Vec3dArray>> &result)
{
	typedef Coordinate PT;
	geos::geom::GeometryFactory factory;
	geos::geom::CoordinateArraySequenceFactory csf;
	int num1 = polygon1->size();
	if (num1 < 3)
		return false;

	geos::geom::CoordinateSequence* cs1 = csf.create(num1 + 1, 3);
	for (int i = 0; i < num1; i++)
	{
		cs1->setAt(Coordinate(polygon1->at(i).x(), polygon1->at(i).y(), polygon1->at(i).z()), i);
	}
	cs1->setAt(Coordinate(polygon1->at(0).x(), polygon1->at(0).y(), polygon1->at(0).z()), num1);
	geos::geom::LinearRing* ring1 = factory.createLinearRing(cs1); //点构成线
	geos::geom::Geometry* pol1 = factory.createPolygon(ring1, NULL);
	geos::geom::Geometry* pol3 = pol1;

	int polygonnum = polygon2.size();
	if (polygonnum < 1)
		return false;

	for (int i = 0; i < polygonnum; i++)
	{
		int num2 = polygon2[i]->size();
		if (num2 < 3)
			continue;

		geos::geom::CoordinateSequence* cs2 = csf.create(num2 + 1, 3);
		for (int j = 0; j < num2; j++)
		{
			cs2->setAt(Coordinate(polygon2[i]->at(j).x(), polygon2[i]->at(j).y(), polygon2[i]->at(j).z()), j);
		}
		cs2->setAt(Coordinate(polygon2[i]->at(0).x(), polygon2[i]->at(0).y(), polygon2[i]->at(0).z()), num2);
		geos::geom::LinearRing* ring2 = factory.createLinearRing(cs2); //点构成线
		geos::geom::Geometry* pol2 = factory.createPolygon(ring2, NULL);

		if (!(pol3->intersects(pol2)))
		{
			return false;
		}
		pol3 = pol3->intersection(pol2);
	}

	int geonum = pol3->getNumGeometries();
	if (geonum == 0)
		return false;

	for (int j = 0; j < geonum; j++)
	{
		osg::ref_ptr<osg::Vec3dArray> point1 = new osg::Vec3dArray();
		const geos::geom::Geometry* pol4 = pol3->getGeometryN(j);
		const geos::geom::CoordinateSequence *a = pol4->getCoordinates();
		int temp = a->getSize();
		for (int i = a->getSize() - 1; i >0; i--)
		{
			point1->push_back(osg::Vec3d(a->getAt(i).x, a->getAt(i).y, a->getAt(i).z));
			cout << a->getAt(i).x << " " << a->getAt(i).y << " " << a->getAt(i).z << endl;
		}
		cout << endl;
		result.push_back(point1);
	}
	return true;
}

//计算polygon1与vector中的多边形两两求交结果 A∩B、A∩C...
bool PolygonIntersection1(const osg::ref_ptr<osg::Vec3dArray>& polygon1, const vector<osg::ref_ptr<osg::Vec3dArray>>& polygon2, vector<osg::ref_ptr<osg::Vec3dArray>> &result)
{
	typedef Coordinate PT;
	geos::geom::GeometryFactory factory;
	geos::geom::CoordinateArraySequenceFactory csf;
	int num1 = polygon1->size();
	if (num1 < 3)
		return false;

	geos::geom::CoordinateSequence* cs1 = csf.create(num1 + 1, 3);
	for (int i = 0; i < num1; i++)
	{
		cs1->setAt(Coordinate(polygon1->at(i).x(), polygon1->at(i).y(), polygon1->at(i).z()), i);
	}
	cs1->setAt(Coordinate(polygon1->at(0).x(), polygon1->at(0).y(), polygon1->at(0).z()), num1);
	geos::geom::LinearRing* ring1 = factory.createLinearRing(cs1); //点构成线
	geos::geom::Geometry* pol1 = factory.createPolygon(ring1, NULL);

	int polygonnum = polygon2.size();
	if (polygonnum < 1)
		return false;

	geos::geom::Geometry* pol3;
	for (int i = 0; i < polygonnum; i++)
	{
		int num2 = polygon2[i]->size();
		if (num2 < 3)
			continue;

		geos::geom::CoordinateSequence* cs2 = csf.create(num2 + 1, 3);
		for (int j = 0; j < num2; j++)
		{
			cs2->setAt(Coordinate(polygon2[i]->at(j).x(), polygon2[i]->at(j).y(), polygon2[i]->at(j).z()), j);
		}
		cs2->setAt(Coordinate(polygon2[i]->at(0).x(), polygon2[i]->at(0).y(), polygon2[i]->at(0).z()), num2);
		geos::geom::LinearRing* ring2 = factory.createLinearRing(cs2); //点构成线
		geos::geom::Geometry* pol2 = factory.createPolygon(ring2, NULL);
		
		if (!(pol1->intersects(pol2)))
		{
			continue;
		}
		pol3 = pol1->intersection(pol2);
		int geonum = pol3->getNumGeometries();
		for (int j = 0; j < geonum; j++)
		{
			osg::ref_ptr<osg::Vec3dArray> point1 = new osg::Vec3dArray();
			const geos::geom::Geometry* pol4 = pol3->getGeometryN(j);
			const geos::geom::CoordinateSequence *a = pol4->getCoordinates();
			int temp = a->getSize();
			for (int i = a->getSize() - 1; i >0; i--)
			{
				point1->push_back(osg::Vec3d(a->getAt(i).x, a->getAt(i).y, a->getAt(i).z));
				cout << a->getAt(i).x << " " << a->getAt(i).y << " " << a->getAt(i).z << endl;
			}
			cout << endl;
			result.push_back(point1);
		}
	}
	if (result.size()>0)
		return true;
	else
		return false;
}

//求vector公共交点 A∩B∩C...
bool PolygonIntersection(const vector<osg::ref_ptr<osg::Vec3dArray>>& polygon, vector<osg::ref_ptr<osg::Vec3dArray>> &result)
{
	typedef Coordinate PT;
	geos::geom::GeometryFactory factory;
	geos::geom::CoordinateArraySequenceFactory csf;
	int num1 = polygon.size();
	if (num1 < 1)
		return false;

	int num2 = polygon[0]->size();
	if (num2 < 3)
		return false;

	geos::geom::CoordinateSequence* cs1 = csf.create(num2 + 1, 3);
	for (int j = 0; j < num2; j++)
	{
		cs1->setAt(Coordinate(polygon[0]->at(j).x(), polygon[0]->at(j).y(), polygon[0]->at(j).z()), j);
	}

	cs1->setAt(Coordinate(polygon[0]->at(0).x(), polygon[0]->at(0).y(), polygon[0]->at(0).z()), num2);
	geos::geom::LinearRing* ring1 = factory.createLinearRing(cs1); //点构成线
	geos::geom::Geometry* pol1 = factory.createPolygon(ring1, NULL);

	for (int ii = 1; ii < num1; ii++)
	{
		int num3 = polygon[ii]->size();
		if (num3 < 3)
			continue;

		geos::geom::CoordinateSequence* cs2 = csf.create(num3 + 1, 3);
		for (int jj = 0; jj < num3; jj++)
		{
			cs2->setAt(Coordinate(polygon[ii]->at(jj).x(), polygon[ii]->at(jj).y(), polygon[ii]->at(jj).z()), jj);
		}
		cs2->setAt(Coordinate(polygon[ii]->at(0).x(), polygon[ii]->at(0).y(), polygon[ii]->at(0).z()), num3);
		geos::geom::LinearRing* ring2 = factory.createLinearRing(cs2); //点构成线
		geos::geom::Geometry* pol2 = factory.createPolygon(ring2, NULL);

		if (!(pol1->intersects(pol2)))
		{
			return false;
		}

		pol1 = pol1->intersection(pol2);
	}

	int geonum = pol1->getNumGeometries();

	for (int n = 1; n < geonum; n++)
	{
		osg::ref_ptr<osg::Vec3dArray> point1 = new osg::Vec3dArray();
		const geos::geom::Geometry* pol4 = pol1->getGeometryN(n);
		const geos::geom::CoordinateSequence *a = pol4->getCoordinates();
		int temp = a->getSize();
		for (int nn = a->getSize() - 1; nn >0; nn--)
		{
			point1->push_back(osg::Vec3d(a->getAt(nn).x, a->getAt(nn).y, a->getAt(nn).z));
			cout << a->getAt(nn).x << " " << a->getAt(nn).y << " " << a->getAt(nn).z << endl;
		}
		cout << endl;
		result.push_back(point1);
	}

	if (result.size() != 0)
		return true;
	else
		return false;
}

//两个多边形并集 A∪B
bool PolygonUnion(const osg::ref_ptr<osg::Vec3dArray>& polygon1, const osg::ref_ptr<osg::Vec3dArray>& polygon2, vector<osg::ref_ptr<osg::Vec3dArray>> &result)
{
	typedef Coordinate PT;
	geos::geom::GeometryFactory factory;
	geos::geom::CoordinateArraySequenceFactory csf;
	int num1 = polygon1->size();
	if (num1 < 3)
		return false;

	geos::geom::CoordinateSequence* cs1 = csf.create(num1+1 , 3);
	for (int i = num1-1, j = 0; i >=0; i--)
	{
		cs1->setAt(Coordinate(polygon1->at(i).x(), polygon1->at(i).y(), polygon1->at(i).z()), j++);
		//std::cout << polygon1->at(i).x() << " " << polygon1->at(i).y() << " "<<polygon1->at(i).z() << endl;
	}
	cs1->setAt(Coordinate(polygon1->at(num1-1).x(), polygon1->at(num1-1).y(), polygon1->at(num1-1).z()), num1);
	geos::geom::LinearRing* ring1 = factory.createLinearRing(cs1); //点构成线
	Geometry* pol1 = factory.createPolygon(ring1, NULL);

	int num2 = polygon2->size();
	if (num2 < 3)
		return false;

	geos::geom::CoordinateSequence* cs2 = csf.create(num2+1 , 3);
	for (int i = num2-1, j = 0; i >= 0; i--)
	{
		cs2->setAt(Coordinate(polygon2->at(i).x(), polygon2->at(i).y(), polygon2->at(i).z()), j++);
	}
	cs2->setAt(Coordinate(polygon2->at(num2-1).x(), polygon2->at(num2-1).y(), polygon2->at(num2-1).z()), num2);
	geos::geom::LinearRing* ring2 = factory.createLinearRing(cs2); //点构成线
	geos::geom::Geometry* pol2 = factory.createPolygon(ring2, NULL);

	geos::geom::Geometry* pol3 = pol1->Union(pol2);
	int geonum = pol3->getNumGeometries();
	const Polygon* p = dynamic_cast<const Polygon*>(pol1);

	//int s = p->getNumInteriorRing();
	/*const geos::geom::CoordinateSequence *a1 = p->getInteriorRingN(0)->getCoordinates();
	const geos::geom::CoordinateSequence *a2 = p->getExteriorRing()->getCoordinates();*/

	for (int j = 0; j < geonum; j++)
	{
		osg::ref_ptr<osg::Vec3dArray> point1 = new osg::Vec3dArray();
		const geos::geom::Geometry* pol4 = pol3->getGeometryN(j);
		const Polygon* pol = dynamic_cast<const Polygon*>(pol4);
		int holenum = pol->getNumInteriorRing();
		const geos::geom::CoordinateSequence *a2 = pol->getExteriorRing()->getCoordinates();
		const geos::geom::CoordinateSequence *a = pol4->getCoordinates();
		for (int i = a2->getSize() - 1; i >0; i--)
		{
			point1->push_back(osg::Vec3d(a2->getAt(i).x, a2->getAt(i).y, a2->getAt(i).z));
			cout << setiosflags(ios::fixed) << std::setprecision(6) << a->getAt(i).x << " " << a->getAt(i).y << " " << a->getAt(i).z << endl;
		}
		result.push_back(point1);

		if (holenum > 0)
		{
			for (int i = 0; i < holenum; i++)
			{
				const geos::geom::CoordinateSequence *a1 = pol->getInteriorRingN(i)->getCoordinates();
				osg::ref_ptr<osg::Vec3dArray> point2 = new osg::Vec3dArray();
				cout << "hole " << i << endl;
				for (int ii = a1->getSize() - 1; ii >= 0; ii--)
				{
					point2->push_back(osg::Vec3d(a1->getAt(ii).x, a1->getAt(ii).y, a1->getAt(ii).z));
					cout << setiosflags(ios::fixed) << std::setprecision(6) << a1->getAt(ii).x << " " << a1->getAt(ii).y << " " << a1->getAt(ii).z << endl;
				}
				result.push_back(point2);
			}			
		}		
		cout << endl;
		
	}
	return true;
}

bool PolygonUnion(const osg::ref_ptr<osg::Vec3dArray>& polygon1, const osg::ref_ptr<osg::Vec3dArray>& polygon2, vector<Area> &result)
{
	typedef Coordinate PT;
	const PrecisionModel* pr = new PrecisionModel(1000);
	geos::geom::GeometryFactory* factory=new GeometryFactory(pr);
	geos::geom::CoordinateArraySequenceFactory csf;
	int num1 = polygon1->size();
	if (num1 < 3)
		return false;

	geos::geom::CoordinateSequence* cs1 = csf.create(num1, 3);
	for (int i = num1 - 1, j = 0; i >= 0; i--)
	{
		cs1->setAt(Coordinate(pr->makePrecise(polygon1->at(i).x()), pr->makePrecise(polygon1->at(i).y()), pr->makePrecise(polygon1->at(i).z())), j++);
		//cs1->setAt(Coordinate(floor(polygon1->at(i).x()*1000)/1000, floor(polygon1->at(i).y()*1000)/1000, floor(polygon1->at(i).z()*1000)/1000), j++);
		//std::cout << polygon1->at(i).x() << " " << polygon1->at(i).y() << " "<<polygon1->at(i).z() << endl;
	}
	//cs1->setAt(Coordinate(polygon1->at(num1 - 1).x(), polygon1->at(num1 - 1).y(), polygon1->at(num1 - 1).z()), num1);
	geos::geom::LinearRing* ring1 = factory->createLinearRing(cs1); //点构成线
	Geometry* pol1 = factory->createPolygon(ring1, NULL);
	/*Geometry* buff= pol1->buffer(0.1);
	buff = buff->buffer(-0.1);
	
	const geos::geom::CoordinateSequence* aa1 = buff->getCoordinates();
	for (int i = 0; i < aa1->size(); i++)
	{
		cout << setiosflags(ios::fixed) << std::setprecision(6) << aa1->getAt(i).x << " " << aa1->getAt(i).y << " " << aa1->getAt(i).z << endl;
	}*/


	int num2 = polygon2->size();
	if (num2 < 3)
		return false;

	geos::geom::CoordinateSequence* cs2 = csf.create(num2, 3);
	for (int i = num2 - 1, j = 0; i >= 0; i--)
	{
		double t = pr->makePrecise(polygon2->at(i).y());
		cs2->setAt(Coordinate(pr->makePrecise(polygon2->at(i).x()), pr->makePrecise(polygon2->at(i).y()), pr->makePrecise(polygon2->at(i).z())), j++);
		//cs2->setAt(Coordinate(floor(polygon2->at(i).x() * 1000) / 1000, floor(polygon2->at(i).y() * 1000) / 1000, floor(polygon2->at(i).z() * 1000) / 1000), j++);
	}
	//cs2->setAt(Coordinate(polygon2->at(num2 - 1).x(), polygon2->at(num2 - 1).y(), polygon2->at(num2 - 1).z()), num2);
	geos::geom::LinearRing* ring2 = factory->createLinearRing(cs2); //点构成线
	geos::geom::Geometry* pol2 = factory->createPolygon(ring2, NULL);

	geos::geom::Geometry* pol3 = pol1->Union(pol2);
	int geonum = pol3->getNumGeometries();
	const Polygon* p = dynamic_cast<const Polygon*>(pol1);

	//int s = p->getNumInteriorRing();
	/*const geos::geom::CoordinateSequence *a1 = p->getInteriorRingN(0)->getCoordinates();
	const geos::geom::CoordinateSequence *a2 = p->getExteriorRing()->getCoordinates();*/

	for (int j = 0; j < geonum; j++)
	{
		vector<Point1> point1;
		
		const geos::geom::Geometry* pol4 = pol3->getGeometryN(j);
		const geos::geom::CoordinateSequence* aa = pol4->getCoordinates();
		const Polygon* pol = dynamic_cast<const Polygon*>(pol4);
		int holenum = pol->getNumInteriorRing();
		const geos::geom::CoordinateSequence *a2 = pol->getExteriorRing()->getCoordinates();
		const geos::geom::CoordinateSequence *a = pol4->getCoordinates();
		if (holenum == 0)
		{
			for (int i = a2->getSize() - 1; i >=0; i--)
			{
				point1.push_back(osg::Vec3d(a2->getAt(i).x, a2->getAt(i).y, a2->getAt(i).z));
				cout << setiosflags(ios::fixed) << std::setprecision(6) << a->getAt(i).x << " " << a->getAt(i).y << " " << a->getAt(i).z << endl;
			}
			result.push_back(point1); 
		}		
		else
		{
			for (int i = a2->getSize() - 1; i >= 0; i--)
			{
				point1.push_back(osg::Vec3d(a2->getAt(i).x, a2->getAt(i).y, a2->getAt(i).z));
				cout << setiosflags(ios::fixed) << std::setprecision(6) << a2->getAt(i).x << " " << a2->getAt(i).y << " " << a2->getAt(i).z << endl;
			}
			Area area(point1);
			for (int i = 0; i < holenum; i++)
			{
				const geos::geom::CoordinateSequence *a1 = pol->getInteriorRingN(i)->getCoordinates();
				point1.clear();
				cout << "hole " << i << endl;
				for (int ii = a1->getSize() - 1; ii >= 0; ii--)
				{
					point1.push_back(osg::Vec3d(a1->getAt(ii).x, a1->getAt(ii).y, a1->getAt(ii).z));
					cout << setiosflags(ios::fixed) << std::setprecision(6) << a1->getAt(ii).x << " " << a1->getAt(ii).y << " " << a1->getAt(ii).z << endl;
				}
				area.addhole(point1);
			}
			result.push_back(area);
		}

		cout << endl;

	}
	return true;
}

//指定多边形与多个多边形并集 A∪B∪C...
bool PolygonUnion(const osg::ref_ptr<osg::Vec3dArray>& polygon1, const vector<osg::ref_ptr<osg::Vec3dArray>>& polygon2, vector<osg::ref_ptr<osg::Vec3dArray>> &result)
{
	typedef Coordinate PT;
	geos::geom::GeometryFactory factory;
	geos::geom::CoordinateArraySequenceFactory csf;
	int num1 = polygon1->size();
	if (num1 < 3)
		return false;

	geos::geom::CoordinateSequence* cs1 = csf.create(num1 + 1, 3);
	for (int i = num1-1,j=0; i >=0; i--)
	{
		cs1->setAt(Coordinate(polygon1->at(i).x(), polygon1->at(i).y(), polygon1->at(i).z()), j++);
	}
	cs1->setAt(Coordinate(polygon1->at(num1-1).x(), polygon1->at(num1-1).y(), polygon1->at(num1-1).z()), num1);
	geos::geom::LinearRing* ring1 = factory.createLinearRing(cs1); //点构成线
	geos::geom::Geometry* pol1 = factory.createPolygon(ring1, NULL);
	geos::geom::Geometry* pol3 = pol1;

	int polygonnum = polygon2.size();
	if (polygonnum < 1)
		return false;

	for (int i = 0; i < polygonnum; i++)
	{
		int num2 = polygon2[i]->size();
		if (num2 < 3)
			continue;

		geos::geom::CoordinateSequence* cs2 = csf.create(num2 + 1, 3);
		for (int j = num2-1,ii=0; j >=0; j--)
		{
			cs2->setAt(Coordinate(polygon2[i]->at(j).x(), polygon2[i]->at(j).y(), polygon2[i]->at(j).z()), ii++);
		}
		cs2->setAt(Coordinate(polygon2[i]->at(num2-1).x(), polygon2[i]->at(num2-1).y(), polygon2[i]->at(num2-1).z()), num2);
		geos::geom::LinearRing* ring2 = factory.createLinearRing(cs2); //点构成线
		geos::geom::Geometry* pol2 = factory.createPolygon(ring2, NULL);

		pol3 = pol3->Union(pol2);
	}

	int geonum = pol3->getNumGeometries();
	if (geonum == 0)
		return false;

	for (int j = 0; j < geonum; j++)
	{
		osg::ref_ptr<osg::Vec3dArray> point1 = new osg::Vec3dArray();
		const geos::geom::Geometry* pol4 = pol3->getGeometryN(j);
		const geos::geom::CoordinateSequence *a = pol4->getCoordinates();
		int temp = a->getSize();
		//if (pol3->intersects(pol2))
		for (int i = a->getSize() - 1; i >0; i--)
		{
			point1->push_back(osg::Vec3d(a->getAt(i).x, a->getAt(i).y, a->getAt(i).z));
			cout << a->getAt(i).x << " " << a->getAt(i).y << " " << a->getAt(i).z << endl;
		}
		cout << endl;
		result.push_back(point1);
	}
	return true;
}

//多多个边形与多个多边形并集 A∪B∪C...
bool PolygonUnion(const vector<osg::ref_ptr<osg::Vec3dArray>>& polygon, vector<Area> &result)
{
	typedef Coordinate PT;
	geos::geom::GeometryFactory factory;
	geos::geom::CoordinateArraySequenceFactory csf;
	int num1 = polygon.size();
	if (num1 < 1)
		return false;

	if (num1 == 1)
	{
		vector<Point1> geo;
		for (int i = 0; i < polygon[0]->size(); i++)
		{
			geo.push_back(osg::Vec3d(polygon[0]->at(i).x(), polygon[0]->at(i).y(), polygon[0]->at(i).z()));
		}
		result.push_back(geo);
		return true;
	}

	int num2 = polygon[0]->size();
	if (num2 < 3)
		return false;

	geos::geom::CoordinateSequence* cs1 = csf.create(num2 + 1, 3);
	for (int j = num2-1,i=0; j >= 0; j--)
	{
		cs1->setAt(Coordinate(polygon[0]->at(j).x(), polygon[0]->at(j).y(), polygon[0]->at(j).z()), i++);
	}
	
	cs1->setAt(Coordinate(polygon[0]->at(num2-1).x(), polygon[0]->at(num2-1).y(), polygon[0]->at(num2-1).z()), num2);
	geos::geom::LinearRing* ring1 = factory.createLinearRing(cs1); //点构成线
	geos::geom::Geometry* pol1 = factory.createPolygon(ring1, NULL);
	bool f = ring1->isValid(); //判断线段是否是闭合曲线（几何体是否有效）
	for (int ii = 1; ii < num1; ii++)
	{
		int num3 = polygon[ii]->size();
		if (num3 < 3)
			continue;

		geos::geom::CoordinateSequence* cs2 = csf.create(num3 + 1, 3);
		for (int jj = num3-1,i=0; jj >= 0; jj--)
		{
			cs2->setAt(Coordinate(polygon[ii]->at(jj).x(), polygon[ii]->at(jj).y(), polygon[ii]->at(jj).z()), i++);
		}
		cs2->setAt(Coordinate(polygon[ii]->at(num3-1).x(), polygon[ii]->at(num3-1).y(), polygon[ii]->at(num3-1).z()), num3);
		geos::geom::LinearRing* ring2 = factory.createLinearRing(cs2); //点构成线
		geos::geom::Geometry* pol2 = factory.createPolygon(ring2, NULL);

		pol1 = pol1->Union(pol2);
		int geonum = pol1->getNumGeometries();

		/*for (int j = 0; j < geonum; j++)
		{
			vector<Point1> point1;

			const geos::geom::Geometry* pol4 = pol1->getGeometryN(j);
			const geos::geom::CoordinateSequence* aa = pol4->getCoordinates();
			const Polygon* pol = dynamic_cast<const Polygon*>(pol4);
			int holenum = pol->getNumInteriorRing();
			const geos::geom::CoordinateSequence *a2 = pol->getExteriorRing()->getCoordinates();
			const geos::geom::CoordinateSequence *a = pol4->getCoordinates();
			if (holenum == 0)
			{
				for (int i = a2->getSize() - 1; i >0; i--)
				{
					point1.push_back(osg::Vec3d(a2->getAt(i).x, a2->getAt(i).y, a2->getAt(i).z));
					cout << setiosflags(ios::fixed) << std::setprecision(6) << a->getAt(i).x << " " << a->getAt(i).y << " " << a->getAt(i).z << endl;
				}
				result.push_back(point1);
			}
			else
			{
				for (int i = a2->getSize() - 1; i >= 0; i--)
				{
					point1.push_back(osg::Vec3d(a2->getAt(i).x, a2->getAt(i).y, a2->getAt(i).z));
					cout << setiosflags(ios::fixed) << std::setprecision(6) << a2->getAt(i).x << " " << a2->getAt(i).y << " " << a2->getAt(i).z << endl;
				}
				Area area(point1);
				for (int i = 0; i < holenum; i++)
				{
					const geos::geom::CoordinateSequence *a1 = pol->getInteriorRingN(i)->getCoordinates();
					point1.clear();
					cout << "hole " << i << endl;
					for (int ii = a1->getSize() - 1; ii >= 0; ii--)
					{
						point1.push_back(osg::Vec3d(a1->getAt(ii).x, a1->getAt(ii).y, a1->getAt(ii).z));
						cout << setiosflags(ios::fixed) << std::setprecision(6) << a1->getAt(ii).x << " " << a1->getAt(ii).y << " " << a1->getAt(ii).z << endl;
					}
					area.addhole(point1);
				}
				result.push_back(area);
			}
			cout << endl;

		}*/
	}

	int geonum = pol1->getNumGeometries();

	for (int j = 0; j < geonum; j++)
	{
		vector<Point1> point1;

		const geos::geom::Geometry* pol4 = pol1->getGeometryN(j);
		const geos::geom::CoordinateSequence* aa = pol4->getCoordinates();
		const Polygon* pol = dynamic_cast<const Polygon*>(pol4);
		int holenum = pol->getNumInteriorRing();
		const geos::geom::CoordinateSequence *a2 = pol->getExteriorRing()->getCoordinates();
		const geos::geom::CoordinateSequence *a = pol4->getCoordinates();
		if (holenum == 0)
		{
			for (int i = a2->getSize() - 1; i >0; i--)
			{
				point1.push_back(osg::Vec3d(a2->getAt(i).x, a2->getAt(i).y, a2->getAt(i).z));
				cout << setiosflags(ios::fixed) << std::setprecision(6) << a->getAt(i).x << " " << a->getAt(i).y << " " << a->getAt(i).z << endl;
			}
			result.push_back(point1);
		}
		else
		{
			for (int i = a2->getSize() - 1; i >= 0; i--)
			{
				point1.push_back(osg::Vec3d(a2->getAt(i).x, a2->getAt(i).y, a2->getAt(i).z));
				cout << setiosflags(ios::fixed) << std::setprecision(6) << a2->getAt(i).x << " " << a2->getAt(i).y << " " << a2->getAt(i).z << endl;
			}
			Area area(point1);
			for (int i = 0; i < holenum; i++)
			{
				const geos::geom::CoordinateSequence *a1 = pol->getInteriorRingN(i)->getCoordinates();
				point1.clear();
				cout << "hole " << i << endl;
				for (int ii = a1->getSize() - 1; ii >= 0; ii--)
				{
					point1.push_back(osg::Vec3d(a1->getAt(ii).x, a1->getAt(ii).y, a1->getAt(ii).z));
					cout << setiosflags(ios::fixed) << std::setprecision(6) << a1->getAt(ii).x << " " << a1->getAt(ii).y << " " << a1->getAt(ii).z << endl;
				}
				area.addhole(point1);
			}
			result.push_back(area);
		}

		cout << endl;

	}

	if (result.size() != 0)
		return true;
	else
		return false;
}

//geos::geom::Geometry* PolygonUnion(const vector<osg::ref_ptr<osg::Vec3dArray>>& polygon, geos::geom::Geometry* a)
//{
//	typedef Coordinate PT;
//	geos::geom::GeometryFactory factory;
//	geos::geom::CoordinateArraySequenceFactory csf;
//	int num1 = polygon.size();
//	if (num1 < 1)
//		return false;
//
//	int num2 = polygon[0]->size();
//	if (num2 < 3)
//		return false;
//
//	geos::geom::CoordinateSequence* cs1 = csf.create(num2 + 1, 3);
//	for (int j = 0; j < num2; j++)
//	{
//		cs1->setAt(Coordinate(polygon[0]->at(j).x(), polygon[0]->at(j).y(), polygon[0]->at(j).z()), j);
//	}
//
//	cs1->setAt(Coordinate(polygon[0]->at(0).x(), polygon[0]->at(0).y(), polygon[0]->at(0).z()), num2);
//	geos::geom::LinearRing* ring1 = factory.createLinearRing(cs1); //点构成线
//	geos::geom::Geometry* pol1 = factory.createPolygon(ring1, NULL);
//
//	for (int ii = 1; ii < num1; ii++)
//	{
//		int num3 = polygon[ii]->size();
//		if (num3 < 3)
//			continue;
//
//		geos::geom::CoordinateSequence* cs2 = csf.create(num3 + 1, 3);
//		for (int jj = 0; jj < num3; jj++)
//		{
//			cs2->setAt(Coordinate(polygon[ii]->at(jj).x(), polygon[ii]->at(jj).y(), polygon[ii]->at(jj).z()), jj);
//		}
//		cs2->setAt(Coordinate(polygon[ii]->at(0).x(), polygon[ii]->at(0).y(), polygon[ii]->at(0).z()), num3);
//		geos::geom::LinearRing* ring2 = factory.createLinearRing(cs2); //点构成线
//		geos::geom::Geometry* pol2 = factory.createPolygon(ring2, NULL);
//		pol1 = pol1->Union(pol2);
//	}
//
//	int geonum = pol1->getNumGeometries();
//
//	for (int n = 0; n < geonum; n++)
//	{
//		osg::ref_ptr<osg::Vec3dArray> point1 = new osg::Vec3dArray();
//		const geos::geom::Geometry* pol4 = pol1->getGeometryN(n);
//		const geos::geom::CoordinateSequence *a = pol4->getCoordinates();
//		int temp = a->getSize();
//		for (int nn = 0; nn < a->getSize() - 1; nn++)
//		{
//			point1->push_back(osg::Vec3d(a->getAt(nn).x, a->getAt(nn).y, a->getAt(nn).z));
//			cout << a->getAt(nn).x << " " << a->getAt(nn).y << " " << a->getAt(nn).z << endl;
//		}
//		cout << endl;
//	}
//	a = pol1->clone();
//	auto_ptr<Geometry> b;
//	b.reset(pol1->getFactory()->createPolygon());
//	pol1->~Geometry();
//	return b.release();
//}

//多边形对称差异，即属于A或属于B A∪B-A∩B
bool PolygonsymDifference(const osg::ref_ptr<osg::Vec3dArray>& polygon1, const osg::ref_ptr<osg::Vec3dArray>& polygon2, vector<osg::ref_ptr<osg::Vec3dArray>> &result)
{
	typedef Coordinate PT;
	geos::geom::GeometryFactory factory;
	geos::geom::CoordinateArraySequenceFactory csf;
	int num1 = polygon1->size();
	if (num1 < 3)
		return false;

	geos::geom::CoordinateSequence* cs1 = csf.create(num1 + 1, 3);
	for (int i = num1-1,j = 0; i >= 0; i--)
	{
		cs1->setAt(Coordinate(polygon1->at(i).x(), polygon1->at(i).y(), polygon1->at(i).z()), j++);
		//std::cout << polygon1->at(i).x() << " " << polygon1->at(i).y() << " "<<polygon1->at(i).z() << endl;
	}

	cs1->setAt(Coordinate(polygon1->at(num1-1).x(), polygon1->at(num1-1).y(), polygon1->at(num1-1).z()), num1);
	geos::geom::LinearRing* ring1 = factory.createLinearRing(cs1); //点构成线
	geos::geom::Geometry* pol1 = factory.createPolygon(ring1, NULL);

	int num2 = polygon2->size();
	if (num2 < 3)
		return false;

	geos::geom::CoordinateSequence* cs2 = csf.create(num2 + 1, 3);
	for (int i = num2-1, j = 0; i >= 0; i--)
	{
		cs2->setAt(Coordinate(polygon2->at(i).x(), polygon2->at(i).y(), polygon2->at(i).z()), j++);
	}
	cs2->setAt(Coordinate(polygon2->at(num2-1).x(), polygon2->at(num2-1).y(), polygon2->at(num2-1).z()), num2);
	geos::geom::LinearRing* ring2 = factory.createLinearRing(cs2); //点构成线
	geos::geom::Geometry* pol2 = factory.createPolygon(ring2, NULL);

	/*if (pol1->intersects(pol2) == false)
	{
		return false;
	}*/
	geos::geom::Geometry* pol3 = pol1->symDifference(pol2);
	int geonum = pol3->getNumGeometries();

	for (int j = 0; j < geonum; j++)
	{
		osg::ref_ptr<osg::Vec3dArray> point1 = new osg::Vec3dArray();
		const geos::geom::Geometry* pol4 = pol3->getGeometryN(j);
		const geos::geom::CoordinateSequence *a = pol4->getCoordinates();
		int temp = a->getSize();
		for (int i = a->getSize() - 1; i >0; i--)
		{
			point1->push_back(osg::Vec3d(a->getAt(i).x, a->getAt(i).y, a->getAt(i).z));
			cout << a->getAt(i).x << " " << a->getAt(i).y << " " << a->getAt(i).z << endl;
		}
		cout << endl;
		result.push_back(point1);
	}
	return true;
}

//多边形对称差异，即属于A或属于B A∪B∪C-A∩B-A∩B-B∩C
bool PolygonsymDifference(const osg::ref_ptr<osg::Vec3dArray>& polygon1, const vector<osg::ref_ptr<osg::Vec3dArray>>& polygon2, vector<Area> &result)
{
	typedef Coordinate PT;
	geos::geom::GeometryFactory factory;
	geos::geom::CoordinateArraySequenceFactory csf;
	int num1 = polygon1->size();
	if (num1 < 3)
		return false;

	geos::geom::CoordinateSequence* cs1 = csf.create(num1 + 1, 3);
	for (int i = num1-1, j=0; i >= 0; i--)
	{
		cs1->setAt(Coordinate(polygon1->at(i).x(), polygon1->at(i).y(), polygon1->at(i).z()), j++);
	}
	cs1->setAt(Coordinate(polygon1->at(num1-1).x(), polygon1->at(num1-1).y(), polygon1->at(num1-1).z()), num1);
	geos::geom::LinearRing* ring1 = factory.createLinearRing(cs1); //点构成线
	geos::geom::Geometry* pol1 = factory.createPolygon(ring1, NULL);
	
	int num2 = polygon2.size();
	if (num2 < 1)
		return false;

	int num3 = polygon2[0]->size();
	if (num3 < 3)
		return false;

	geos::geom::CoordinateSequence* cs2 = csf.create(num3 + 1, 3);
	for (int j = num3-1, i=0; j >= 0; j--)
	{
		cs2->setAt(Coordinate(polygon2[0]->at(j).x(), polygon2[0]->at(j).y(), polygon2[0]->at(j).z()), i++);
	}

	cs2->setAt(Coordinate(polygon2[0]->at(num3-1).x(), polygon2[0]->at(num3-1).y(), polygon2[0]->at(num3-1).z()), num3);
	geos::geom::LinearRing* ring2 = factory.createLinearRing(cs2); //点构成线
	geos::geom::Geometry* pol2 = factory.createPolygon(ring2, NULL);

	for (int ii = 1; ii < num2; ii++)
	{
		int num3 = polygon2[ii]->size();
		if (num3 < 3)
			continue;

		geos::geom::CoordinateSequence* cs3 = csf.create(num3 + 1, 3);
		for (int jj = 0; jj < num3; jj++)
		{
			cs3->setAt(Coordinate(polygon2[ii]->at(jj).x(), polygon2[ii]->at(jj).y(), polygon2[ii]->at(jj).z()), jj);
		}
		cs3->setAt(Coordinate(polygon2[ii]->at(0).x(), polygon2[ii]->at(0).y(), polygon2[ii]->at(0).z()), num3);
		geos::geom::LinearRing* ring3 = factory.createLinearRing(cs3); //点构成线
		geos::geom::Geometry* pol3 = factory.createPolygon(ring3, NULL);
		pol2 = pol2->Union(pol3);
	}

	pol1 = pol1->symDifference(pol2);
	int geonum = pol1->getNumGeometries();
	/*const geos::geom::Polygon* pol = dynamic_cast<const geos::geom::Polygon*>(pol1->getGeometryN(0));*/

	for (int j = 0; j < geonum; j++)
	{
		vector<Point1> point1;

		const geos::geom::Geometry* pol4 = pol1->getGeometryN(j);
		const Polygon* pol = dynamic_cast<const Polygon*>(pol4);
		int holenum = pol->getNumInteriorRing();
		const geos::geom::CoordinateSequence *a2 = pol->getExteriorRing()->getCoordinates();
		if (holenum == 0)
		{
			for (int i = a2->getSize() - 1; i >0; i--)
			{
				point1.push_back(osg::Vec3d(a2->getAt(i).x, a2->getAt(i).y, a2->getAt(i).z));
				//cout << setiosflags(ios::fixed) << std::setprecision(6) << a->getAt(i).x << " " << a->getAt(i).y << " " << a->getAt(i).z << endl;
			}
			result.push_back(point1);
		}
		else
		{
			for (int i = a2->getSize() - 1; i >= 0; i--)
			{
				point1.push_back(osg::Vec3d(a2->getAt(i).x, a2->getAt(i).y, a2->getAt(i).z));
				cout << setiosflags(ios::fixed) << std::setprecision(6) << a2->getAt(i).x << " " << a2->getAt(i).y << " " << a2->getAt(i).z << endl;
			}
			Area area(point1);
			for (int i = 0; i < holenum; i++)
			{
				const geos::geom::CoordinateSequence *a1 = pol->getInteriorRingN(i)->getCoordinates();
				point1.clear();
				cout << "hole " << i << endl;
				for (int ii = a1->getSize() - 1; ii >= 0; ii--)
				{
					point1.push_back(osg::Vec3d(a1->getAt(ii).x, a1->getAt(ii).y, a1->getAt(ii).z));
					cout << setiosflags(ios::fixed) << std::setprecision(6) << a1->getAt(ii).x << " " << a1->getAt(ii).y << " " << a1->getAt(ii).z << endl;
				}
				area.addhole(point1);
			}
			result.push_back(area);
		}
		cout << endl;
	}
	return true;
}

////多边形对称差异，即属于A或属于B 
//bool PolygonsymDifference(const osg::ref_ptr<osg::Vec3dArray>& polygon1, const vector<osg::ref_ptr<osg::Vec3dArray>>& polygon2, vector<osg::ref_ptr<osg::Vec3dArray>> &result)
//{
//	typedef Coordinate PT;
//	geos::geom::GeometryFactory factory;
//	geos::geom::CoordinateArraySequenceFactory csf;
//	int num1 = polygon1->size();
//	if (num1 < 3)
//		return false;
//
//	geos::geom::CoordinateSequence* cs1 = csf.create(num1 + 1, 3);
//	for (int i = 0; i < num1; i++)
//	{
//		cs1->setAt(Coordinate(polygon1->at(i).x(), polygon1->at(i).y(), polygon1->at(i).z()), i);
//	}
//	cs1->setAt(Coordinate(polygon1->at(0).x(), polygon1->at(0).y(), polygon1->at(0).z()), num1);
//	geos::geom::LinearRing* ring1 = factory.createLinearRing(cs1); //点构成线
//	geos::geom::Geometry* pol1 = factory.createPolygon(ring1, NULL);
//	geos::geom::Geometry* pol3 = pol1;
//
//	int polygonnum = polygon2.size();
//	if (polygonnum < 1)
//		return false;
//
//	for (int i = 0; i < polygonnum; i++)
//	{
//		int num2 = polygon2[i]->size();
//		if (num2 < 3)
//			continue;
//
//		geos::geom::CoordinateSequence* cs2 = csf.create(num2 + 1, 3);
//		for (int j = 0; j < num2; j++)
//		{
//			cs2->setAt(Coordinate(polygon2[i]->at(j).x(), polygon2[i]->at(j).y(), polygon2[i]->at(j).z()), j);
//		}
//		cs2->setAt(Coordinate(polygon2[i]->at(0).x(), polygon2[i]->at(0).y(), polygon2[i]->at(0).z()), num2);
//		geos::geom::LinearRing* ring2 = factory.createLinearRing(cs2); //点构成线
//		geos::geom::Geometry* pol2 = factory.createPolygon(ring2, NULL);
//
//		/*if (pol3->intersects(pol2) == false)
//		{
//			continue;
//		}*/
//		pol3 = pol3->symDifference(pol2);
//	}
//
//	int geonum = pol3->getNumGeometries();
//	if (geonum == 0)
//		return false;
//
//	for (int j = 0; j < geonum; j++)
//	{
//		osg::ref_ptr<osg::Vec3dArray> point1 = new osg::Vec3dArray();
//		const geos::geom::Geometry* pol4 = pol3->getGeometryN(j);
//		const geos::geom::CoordinateSequence *a = pol4->getCoordinates();
//		int temp = a->getSize();
//		for (int i = 0; i < a->getSize()-1; i++)
//		{
//			point1->push_back(osg::Vec3d(a->getAt(i).x, a->getAt(i).y, a->getAt(i).z));
//			cout << a->getAt(i).x << " " << a->getAt(i).y << " " << a->getAt(i).z << endl;
//		}
//		cout << endl;
//		result.push_back(point1);
//	}
//	return true;
//}

double StringToDouble(const std::string& s)
{
	std::istringstream i(s);
	double x;
	if (!(i >> x))
		return 0;
	return x;
}

void Readdata(string path1, osg::ref_ptr<osg::Vec3dArray> &polygon1)
{
	string line;
	ifstream myfile(path1);
	if (myfile.is_open())
	{
		int i = 0;
		while (!myfile.eof())
		{
			getline(myfile, line);
			if (line.size() == 0)
			{
				break;
			}
			istringstream iss(line);
			std::vector<string> tokens;
			copy(istream_iterator<string>(iss), istream_iterator<string>(), back_inserter<std::vector<string> >(tokens));
			double x = StringToDouble(tokens[0]);
			double y = StringToDouble(tokens[1]);
			double z = StringToDouble(tokens[2]);
			polygon1->push_back(osg::Vec3d(x, y, z));
		}
	}
	myfile.close();
}

void Writedata(string path1, vector<osg::ref_ptr<osg::Vec3dArray>> result)
{
	ofstream myfile(path1);
	for (int i = 0; i < result.size(); i++)
	{
		for (int j = 0; j < result[i]->size(); j++)
			myfile << setiosflags(ios::fixed) << setprecision(5)<< result[i]->at(j).x() << " " << result[i]->at(j).y() << " " << result[i]->at(j).z() << endl;
		myfile << endl;
	}
	myfile.close();
}

void Writedata(string path1, vector<Area> result)
{
	ofstream myfile(path1);
	for (int i = 0; i < result.size(); i++)
	{
		myfile << i << endl;
		for (int j = 0; j < result[i].poly.size(); j++)
			myfile << setiosflags(ios::fixed) << setprecision(5) << result[i].poly.at(j).xyz.x() << " " << result[i].poly.at(j).xyz.y() << " " << result[i].poly.at(j).xyz.z() << endl;
		if (result[i].hole.size()>0)
		{
			for (int j = 0; j < result[i].hole.size(); j++)
			{
				myfile << "hole " << j << endl;
				for(int n = 0; n < result[i].hole[j].size(); n++)
				{
					myfile << setiosflags(ios::fixed) << setprecision(5) << result[i].hole[j][n].xyz.x() << " " << result[i].hole[j][n].xyz.y() << " " << result[i].hole[j][n].xyz.z() << endl;
				}
			}				
		}
	}
	myfile.close();
}

bool PointinPolygon(const osg::ref_ptr<osg::Vec3dArray>& polygon1, osg::Vec3d point)
{
	typedef Coordinate PT;
	geos::geom::GeometryFactory factory;
	geos::geom::CoordinateArraySequenceFactory csf;
	int num1 = polygon1->size();
	if (num1 < 3)
		return false;

	geos::geom::CoordinateSequence* cs1 = csf.create(num1 + 1, 3);
	for (int i = num1 - 1, j = 0; i >= 0; i--)
	{
		cs1->setAt(Coordinate(polygon1->at(i).x(), polygon1->at(i).y(), polygon1->at(i).z()), j++);
	}
	cs1->setAt(Coordinate(polygon1->at(num1 - 1).x(), polygon1->at(num1 - 1).y(), polygon1->at(num1 - 1).z()), num1);
	geos::geom::LinearRing* ring1 = factory.createLinearRing(cs1); //点构成线
	geos::geom::Geometry* pol1 = factory.createPolygon(ring1, NULL);

	geos::geom::CoordinateSequence* cs2 = csf.create(1, 3);
	geos::geom::Coordinate p = { point.x(), point.y(), point.z() };
	cs2->setAt(Coordinate(point.x(), point.y(), point.z()), 0);
	geos::geom::Geometry* p2 = factory.createPoint(cs2);
	geos::algorithm::CGAlgorithms ca;
	bool flag = ca.isPointInRing(p, cs1);
	bool flag1 = pol1->contains(p2);
	return flag;
	
}