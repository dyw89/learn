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
	static bool Contour2DEM(const char* shpFilename, const char* rasterFilename, int dfXRes, int dfYRes);//�ȸ���תΪDEM��shpFilenameΪ�ȸ����ļ���(.shp��ʽ�����д洢�̵߳������ֶ�ΪElevation)��rasterFilenameΪ.tif��dem�ļ�����dfXResΪx����ֱ��ʣ�dfYResΪy����ֱ���
	static bool DEM2Contour(const char*  imgPath, const char*  shpPath, int dInterval, std::vector<double> &elevation);//dem����תΪ�ȸ������ݣ�imgPathΪdem�ļ�����.tif��ʽ��,shpPathΪ����ȸ�������.shp��ʽ��,dInterval�ȸ߾࣬elevation�洢�ȸ��߸߳�
};