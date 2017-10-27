#pragma once
#ifdef DEMCONTOUR_EXPORTS
#define DEMCONTOUR_DLL __declspec(dllexport)
#else
#define DEMCONTOUR_DLL __declspec(dllimport)
#endif

#include "gdal_priv.h"
#include "ogr_api.h"
#include "ogr_srs_api.h"
#include "gdal_alg.h"
#include <vector>

const int INIT_AR = 64;
const int AR_INCR = 64;
const int MYNO_DATA = 0;


typedef struct _f_l_a_g
{
	int nrows,ncols,length;
	unsigned char **array;
}FLAG;

typedef struct _n_o_d_e
{
	int r,c;
	float d;
} NODE;

typedef struct link_node
{
	float value;
	float dd;
	int  circle;
	struct link_node *next;
}Linknode;

#define FLAG_SET(flags,row,col)   \
	 (flags)->array[(row)][(col)>>3] |= (1<<((col)&7))

class DEMCONTOUR_DLL CDEMContour
{
public:
	CDEMContour();
	~CDEMContour();
	static FLAG *flag_create(int, int);
	static int flag_destroy(FLAG *);
	static bool Is_nodata(float);
	static void free_cell(float **);
	static void find_con(int, int, float *, float *, float *, float *, int *);
	static void delete_link();
	static bool Rasterize(const char* shpFilename, const char* rasterFilename, int dfXRes, int dfYRes);
	static Linknode *create_empty_link();	
	static int flag_get(FLAG * flags, int row, int col);
	static void insert_link(Linknode *head, float value, float dd, int *num_con, float *shortest, int *circle);
	static bool Is_finded(float value);
	static NODE *add_in(int r, int c, int rr, int cc, NODE *zero, int *node_ct, int *circle);
	static NODE *addpts(NODE* zero, int r, int c, int rr, int cc, int *node_ct, int *circle);
	static void find2con(float *d1, float *con1, float *d2, float *con2, int *state);
	static bool Contour2DEM(const char* shpFilename, const char* rasterFilename, int dfXRes, int dfYRes);//等高线转为DEM，shpFilename为等高线文件名(.shp格式，其中存储高程的属性字段为Elevation)，rasterFilename为.tif的dem文件名，dfXRes为x方向分辨率，dfYRes为y方向分辨率
	static bool DEM2Contour(const char*  imgPath, const char*  shpPath, int dInterval, std::vector<double> &elevation);//dem数据转为等高线数据，imgPath为dem文件名（.tif格式）,shpPath为输出等高线名（.shp格式）,dInterval等高距，elevation存储等高线高程
};