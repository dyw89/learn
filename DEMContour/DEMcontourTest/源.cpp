// ContourDEM.cpp : 定义控制台应用程序的入口点。
//

#include "contour.h"
#include <vector>
#include <iostream>
#include <math.h>
#include <float.h>
#include <time.h>

int main()
{
	char *pszSrcFilename = "F:\\ConDem\\contour.shp";
	char *pszDstFilename = "F:\\contour.tif";
	char *imgPath = "F:\\ConDem\\dem.tif";
	char *shpPath = "F:\\ConDem\\contour5.shp";
	std::vector<double> elevation;
	CDEMContour::Contour2DEM(pszSrcFilename, pszDstFilename, 10, 10);
	CDEMContour::DEM2Contour(imgPath, shpPath, 5, elevation);
	std::cout << "OK" << std::endl;
	return 0;
}