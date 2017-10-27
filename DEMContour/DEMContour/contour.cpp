#include "stdafx.h"
#include <iostream>  
#include "gdal_priv.h"  
#include "ogrsf_frmts.h"  
#include "gdal_alg.h"  
#include "contour.h"

using namespace std;


float **con;
FLAG *seen;
NODE *zero1, *zero2;
Linknode *head = NULL;
int nrows, ncols;
int array_size1, array_size2;

CDEMContour::CDEMContour()
{
}

CDEMContour::~CDEMContour()
{
}

bool CDEMContour::Contour2DEM(const char* shpFilename, const char* rasterFilename, int dfXRes, int dfYRes)
{
	int i, j;
	int b3D = FALSE;
	int bInverse = FALSE;
	char **papszLayers = NULL;
	const char *pszBurnAttribute = NULL;
	const char *pszFormat = "GTiff";
	std::vector<int> anBandList;
	std::vector<double> adfInitVals;
	std::vector<double> adfBurnValues;
	char **papszRasterizeOptions = NULL;
	GDALDriverH hDriver = NULL;
	GDALDataType eOutputType = GDT_Float32;
	int nXSize = 0, nYSize = 0;
	int bCreateOutput = FALSE;
	int bNoDataSet = FALSE;
	float dfNoData = 0;
	OGREnvelope sEnvelop;
	int bGotBounds = FALSE;
	OGRSpatialReferenceH hSRS = NULL;
	int bTargetAlignedPixels = FALSE;
	char **papszCreateOptions = NULL;
	GDALProgressFunc pfnProgress = GDALTermProgress;
	int comm_sz = 1;
	int my_rank = 0;

	OGRRegisterAll();
	GDALAllRegister();
	CPLSetConfigOption("GDAL_FILENAME_IS_UTF8", "NO");
	Rasterize(shpFilename, rasterFilename, dfXRes, dfYRes);

	//GRASS进行DEM生成
	GDALDataset *poDataset;
	GDALRasterBand  *poBand;
	float con1, con2;
	float d1, d2;
	float *alt_row;
	int state;//判断两条等高线的相对位置
	int interval = 10;
	float small_value = 0.01;//自定义阈值
	unsigned char *pabyChunkBuf;
	int  iY, iY2;
	int  nYChunkSize, nScanlineBytes, rank_chunkSize, mod_value;
	double time6, time7, time8, time9, time10, time11, time12, time13, time14;

	GDALAllRegister();
	poDataset = (GDALDataset*)GDALOpen(rasterFilename, GA_Update);

	poBand = poDataset->GetRasterBand(1);
	nrows = poDataset->GetRasterYSize();
	ncols = poDataset->GetRasterXSize();
	array_size1 = array_size2 = INIT_AR;
	seen = flag_create(nrows, ncols);

	con = (float **)malloc(nrows * sizeof(float *));
	for (i = 0; i < nrows; i++)
	{
		con[i] = (float *)malloc(ncols * sizeof(float));
	}
	nScanlineBytes = ncols * sizeof(float);
	nYChunkSize = 10000000 / nScanlineBytes;
	if (nYChunkSize > nrows)
		nYChunkSize = nrows;

	pabyChunkBuf = (unsigned char *)VSIMalloc(nYChunkSize * nScanlineBytes);
	if (pabyChunkBuf == NULL)
	{
		CPLError(CE_Failure, CPLE_OutOfMemory,
			"Unable to allocate rasterization buffer.");
		return NULL;
	}

	CPLErr  eErr = CE_None;
	int row;
	for (iY = 0; iY < nrows && eErr == CE_None; iY += nYChunkSize)
	{
		int	nThisYChunkSize;

		nThisYChunkSize = nYChunkSize;
		if (nThisYChunkSize + iY > nrows)
			nThisYChunkSize = nrows - iY;


		eErr =
			poBand->RasterIO(GF_Read, 0, iY, ncols, nThisYChunkSize,
			pabyChunkBuf, ncols, nThisYChunkSize, GDT_Float32, 0, 0);
		if (eErr != CE_None)
			break;

		for (i = 0; i < nThisYChunkSize; i++)
		{
			row = i + iY;
			for (j = 0; j < ncols; j++)
			{
				con[row][j] = *((float *)pabyChunkBuf + i * ncols + j);
			}
		}
	}
	zero1 = (NODE *)malloc(INIT_AR*sizeof(NODE));
	zero2 = (NODE *)malloc(INIT_AR*sizeof(NODE));
	alt_row = (float *)malloc(ncols*sizeof(float));
	head = create_empty_link();//创建头链表

	for (int r = 0; r<nrows; r++)
	{
		for (int c = 0; c<ncols; c++)
		{
			if (Is_nodata(con[r][c]))
			{
				find_con(r, c, &d1, &d2, &con1, &con2, &state);

				if (state == 1)
				{
					alt_row[c] = d2*con1 / (d1 + d2) + d1*con2 / (d1 + d2);

				}
				else if (state == 2)
				{
					alt_row[c] = d1 * con2 / (d1 - d2) - d2 * con1 / (d1 - d2);
					if (alt_row[c] - con1 > interval - small_value)
						alt_row[c] = con1 + interval - small_value;
				}

				//链表值赋值为NO_DATA
				Linknode *temp = head->next;
				while (temp != NULL)
				{
					temp->value = (float)MYNO_DATA;
					temp = temp->next;
				}
			}
			else
			{
				alt_row[c] = con[r][c];
			}

		}
		poBand->RasterIO(GF_Write, 0, r, ncols, 1, alt_row, ncols, 1, GDT_Float32, 0, 0);
	}

	free_cell(con);
	free(alt_row);
	free(zero1);
	free(zero2);
	flag_destroy(seen);
	GDALClose((GDALDatasetH)poDataset);
	delete_link();//释放内存空间
	return true;
}

FLAG *CDEMContour::flag_create(int nrows, int ncols)
{
	unsigned char *temp;
	FLAG *new_flag;
	register int i;

	new_flag = (FLAG *)malloc(sizeof(FLAG));
	if (new_flag == NULL)
	{
		return((FLAG*)NULL);
	}
	new_flag->nrows = nrows;
	new_flag->ncols = ncols;
	new_flag->length = (ncols + 7) / 8;
	new_flag->array = (unsigned char **)malloc(nrows*sizeof(unsigned char *));
	if (new_flag->array == NULL)
	{
		free(new_flag);
		return ((FLAG *)NULL);
	}
	temp = (unsigned char *)calloc(nrows*new_flag->length, sizeof(unsigned char));
	if (temp == NULL)
	{
		free(new_flag->array);
		free(new_flag);
		return ((FLAG *)NULL);
	}
	for (i = 0; i<nrows; i++)
	{
		new_flag->array[i] = temp;
		temp += new_flag->length;
	}
	return new_flag;
}
int CDEMContour::flag_destroy(FLAG *flags)
{
	free(flags->array[0]);
	free(flags->array);
	free(flags);

	return 0;
}

bool CDEMContour::Is_nodata(float value)
{
	if (value == MYNO_DATA)
		return true;
	else
		return false;
}


void CDEMContour::free_cell(float **idx)
{
	for (int row = 0; row<nrows; row++)
		free(idx[row]);
	free(idx);
}

int CDEMContour::flag_get(FLAG * flags, int row, int col)
{
	return (flags->array[row][col >> 3] & (1 << (col & 7)));
}

//链表操作
Linknode *CDEMContour::create_empty_link()
{
	Linknode *head;
	head = (Linknode *)malloc(sizeof(Linknode));
	if (head != NULL)
	{
		head->value = MYNO_DATA;
		head->dd = 0.0;
		head->circle = 0;
		head->next = NULL;
		return head;
	}
	else
		return NULL;
}

void CDEMContour::insert_link(Linknode *head, float value, float dd, int *num_con, float *shortest, int *circle)//按距离升序排列的链表
{
	Linknode *p = head->next;
	bool finded = false;
	while (p != NULL)
	{
		if (p->value == MYNO_DATA)
		{
			break;
		}
		if (value == p->value)
		{
			finded = true;
			if (dd < p->dd)
			{
				p->dd = dd;
			}
			break;
		}
		p = p->next;
	}
	if (!finded)
	{
		p = head;
		Linknode *temp, *temp1;
		temp = head->next;
		temp1 = head;
		while ((temp != NULL) && (temp->value != MYNO_DATA))
		{
			temp1 = temp;
			temp = temp->next;
		}

		if (temp == NULL)
		{
			temp = (Linknode *)malloc(sizeof(Linknode));
		}
		else
		{
			temp1->next = temp->next;//将结点从链中断开
		}

		if (temp != NULL)
		{
			while ((p->next != NULL) && (dd >= p->next->dd))
			{
				if (p->next->value == MYNO_DATA)
				{
					break;
				}
				p = p->next;
			}
			temp->value = value;
			temp->dd = dd;
			temp->circle = *circle;
			temp->next = p->next;
			p->next = temp;
			(*num_con)++;
		}
	}

	if ((*num_con >= 2) && (*circle == 1))//第一轮搜索，找到第二条等高线后，开始用前两条等高线的最大值做限制
	{
		p = head->next;
		float d1 = p->dd;
		float d2 = p->next->dd;
		*shortest = MAX(d1, d2);
	}
	else if (*circle == 2)//第二轮搜索，找到第一条等高线后就做限制
	{
		*shortest = MIN(*shortest, dd);
	}

}

bool CDEMContour::Is_finded(float value)
{
	Linknode *p = head->next;
	while (p != NULL)
	{
		if (p->value == MYNO_DATA)
		{
			break;
		}
		if (p->value == value)
			return true;
		p = p->next;
	}
	return false;
}

void CDEMContour::delete_link()
{
	Linknode *temp = head->next;
	if (temp != NULL)
	{
		head->next = temp->next;
		free(temp);
		temp = head->next;
	}
	free(head);
	head = NULL;
}

//寻找最短的两条等高线
NODE *CDEMContour::add_in(int r, int c, int rr, int cc, NODE *zero, int *node_ct, int *circle)
{
	int dor, doc;
	int *array_size = NULL;
	FLAG_SET(seen, rr, cc);
	if (*circle == 1)
		array_size = &array_size1;
	else
		array_size = &array_size2;
	if (*node_ct == *array_size)
	{
		zero = (NODE *)realloc(zero, (*array_size + AR_INCR)*sizeof(NODE));
		*array_size += AR_INCR;
	}

	dor = ABS(rr - r);
	doc = ABS(cc - c);
	zero[*node_ct].r = rr;
	zero[*node_ct].c = cc;
	zero[*node_ct].d = sqrt((float)(dor*dor + doc*doc));
	*node_ct = *node_ct + 1;
	return zero;
}

NODE *CDEMContour::addpts(NODE* zero, int r, int c, int rr, int cc, int *node_ct, int *circle)
{
	if (rr<nrows - 1)
	{
		if (!flag_get(seen, rr + 1, cc))
			zero = add_in(r, c, rr + 1, cc, zero, node_ct, circle);
	}
	if (cc<ncols - 1)
	{
		if (!flag_get(seen, rr, cc + 1))
			zero = add_in(r, c, rr, cc + 1, zero, node_ct, circle);
	}
	if (rr>0)
	{
		if (!flag_get(seen, rr - 1, cc))
			zero = add_in(r, c, rr - 1, cc, zero, node_ct, circle);
	}
	if (cc>0)
	{
		if (!flag_get(seen, rr, cc - 1))
			zero = add_in(r, c, rr, cc - 1, zero, node_ct, circle);
	}
	return zero;
}

//找到距离最近的两条等高线
void CDEMContour::find2con(float *d1, float *con1, float *d2, float *con2, int *state)
{
	Linknode *p;
	p = head->next;
	*d1 = p->dd;
	*con1 = p->value;
	*d2 = p->next->dd;
	*con2 = p->next->value;
	*state = (p->circle)*(p->next->circle);
}

void CDEMContour::find_con(int r, int c, float *d1, float *d2, float *con1, float *con2, int *state)
{
	int ct1, low_ct1, node_ct1, ct2, low_ct2, node_ct2, num_con;
	int rr, cc;
	int circle;
	float shortest1, shortest2;
	float value;
	Linknode *p = NULL;

	*con1 = *con2 = MYNO_DATA;
	*d1 = *d2 = 1.0;
	num_con = 0;//记录找到的等高线条数
	shortest1 = shortest2 = nrows*ncols;

	if (head != NULL)
	{
		memset(seen->array[0], 0, nrows * seen->length);
		FLAG_SET(seen, r, c);
		node_ct1 = node_ct2 = 0;
		zero1 = addpts(zero1, r, c, r, c, &node_ct1, &circle);
		low_ct1 = 0;
		while (1)
		{
			circle = 1;
			ct1 = low_ct1++;
			if (node_ct1 <= ct1)
			{
				break;
			}
			rr = zero1[ct1].r;
			cc = zero1[ct1].c;
			if (zero1[ct1].d < shortest1)
			{
				value = con[rr][cc];

				if (Is_nodata(value))
				{
					zero1 = addpts(zero1, r, c, rr, cc, &node_ct1, &circle);
					continue;
				}
				//遇到栅格化后的等高线
				insert_link(head, value, zero1[ct1].d, &num_con, &shortest1, &circle);//若不存在，插入新结点，若存在则更新为最短距离，函数返回阈值
				circle = 2;
				zero2 = add_in(r, c, rr, cc, zero2, &node_ct2, &circle);
			}
		}
		if (num_con == 1)//位于山顶或边缘山谷
		{
			low_ct2 = 0;
			while (1)
			{
				circle = 2;
				ct2 = low_ct2++;
				if (node_ct2 <= ct2)
				{
					break;
				}
				rr = zero2[ct2].r;
				cc = zero2[ct2].c;
				if (zero2[ct2].d < shortest2)
				{
					value = con[rr][cc];
					if (Is_nodata(value) || Is_finded(value))
					{
						zero2 = addpts(zero2, r, c, rr, cc, &node_ct2, &circle);
						continue;
					}
					insert_link(head, value, zero2[ct2].d, &num_con, &shortest2, &circle);
				}
			}

			find2con(d1, con1, d2, con2, state);
			if (*con1 < *con2)//只有位于山顶时才进行跨越等高线的内插
			{
				*d1 = 0;
			}
		}
		else
		{
			find2con(d1, con1, d2, con2, state);
		}
	}
}

bool CDEMContour::Rasterize(const char* shpFilename, const char* rasterFilename, int dfXRes, int dfYRes)
{
	GDALAllRegister();
	OGRRegisterAll();
	CPLSetConfigOption("GDAL_FILENAME_IS_UTF8", "NO");

	//打开矢量图层  
	GDALDataset *pOgrSrc = NULL;
	pOgrSrc = (GDALDataset *)GDALOpenEx(shpFilename, GDAL_OF_VECTOR, NULL, NULL, NULL);
	if (pOgrSrc == NULL)
	{
		return false;
	}
	OGRLayer *pOgrLyr;
	pOgrLyr = pOgrSrc->GetLayer(0);

	OGREnvelope env;
	pOgrLyr->GetExtent(&env);
	

	int m_nImageWidth = (env.MaxX - env.MinX) / dfXRes;
	int m_nImageHeight = (env.MaxY - env.MinY) / dfYRes;

	OGRSpatialReference *pOgrSRS = NULL;
	pOgrSRS = pOgrLyr->GetSpatialRef();

	char *pPrj = NULL;
	if (pOgrSRS == NULL)
	{
		m_nImageHeight = (int)env.MaxX;
		m_nImageWidth = (int)env.MaxY;
	}
	else
	{
		pOgrSRS->exportToWkt(&pPrj);
	}

	const char *pszFormat = "GTiff";
	GDALDriver *poDriver = NULL;
	poDriver = GetGDALDriverManager()->GetDriverByName(pszFormat);

	char **papszCreateOptions = NULL;
	papszCreateOptions = CSLSetNameValue(papszCreateOptions, "BLOCKXSIZE", "256");
	papszCreateOptions = CSLSetNameValue(papszCreateOptions, "BLOCKYSIZE", "1");

	GDALDataset *poNewDS = poDriver->Create(rasterFilename,
		m_nImageWidth,
		m_nImageHeight,
		1,
		GDT_Float32,
		papszCreateOptions);

	double adfGeoTransform[6];
	adfGeoTransform[0] = env.MinX;
	adfGeoTransform[1] = dfXRes;
	adfGeoTransform[2] = 0;
	adfGeoTransform[3] = env.MaxY;
	adfGeoTransform[4] = 0;
	adfGeoTransform[5] = -dfYRes;
	GDALSetGeoTransform(poNewDS, adfGeoTransform);
	GDALSetRasterNoDataValue(poNewDS->GetRasterBand(1), 0);

	if (pOgrSRS != NULL)
	{
		poNewDS->SetProjection(pPrj);
	}

	int * pnbandlist = NULL;
	pnbandlist = new int[1];
	pnbandlist[0] = 1;
	double *dburnValues = NULL;
	dburnValues = new double[3];
	dburnValues[0] = 255;
	dburnValues[1] = 111;
	dburnValues[2] = 34;

	OGRLayerH * player;
	player = new OGRLayerH[1];
	player[0] = (OGRLayerH)pOgrLyr;

	char **papszOptions = NULL;
	papszOptions = CSLSetNameValue(papszOptions, "CHUNKSIZE", "1");
	papszOptions = CSLSetNameValue(papszOptions, "ATTRIBUTE", "Elevation");

	void * pTransformArg = NULL;
	void * m_hGenTransformArg = NULL;
	m_hGenTransformArg = GDALCreateGenImgProjTransformer(NULL,
		pPrj,
		(GDALDatasetH)poNewDS,
		poNewDS->GetProjectionRef(),
		false, 1000.0, 3);

	pTransformArg = GDALCreateApproxTransformer(GDALGenImgProjTransform,
		m_hGenTransformArg,
		0.125);

	CPLErr err = GDALRasterizeLayers((GDALDatasetH)poNewDS,
		1,
		pnbandlist,
		1, player,
		GDALGenImgProjTransform,
		m_hGenTransformArg,
		dburnValues,
		papszOptions,
		NULL, //GDALTermProgress
		"vector2raster");

	GDALDestroyGenImgProjTransformer(m_hGenTransformArg);
	GDALDestroyApproxTransformer(pTransformArg);
	GDALClose(poNewDS);

	delete[]player;
	delete[]pnbandlist;
	delete[]dburnValues;

	return true;
}

bool CDEMContour::DEM2Contour(const char *imgPath, const char *shpPath, int dInterval, vector<double> &elevation)
{
	OGRRegisterAll();
	GDALAllRegister();
	CPLSetConfigOption("GDAL_FILENAME_IS_UTF8", "NO");//GDAL处理中文路径

	GDALDataset *pInDataset = (GDALDataset *)GDALOpen(imgPath, GA_ReadOnly);
	if (pInDataset == NULL)
	{
		return FALSE;
	}

	int nDemWidth = pInDataset->GetRasterXSize(); //获取图像宽  
	int nDemHeight = pInDataset->GetRasterYSize(); //获取图像高  
	int Count = pInDataset->GetRasterCount(); //波段数  


	//读取图像数据波段
	GDALRasterBand *pInRasterBand = pInDataset->GetRasterBand(1);
	double nodata = pInRasterBand->GetNoDataValue();
	float *pData = new float[nDemWidth*nDemHeight]();

	CPLErr err = pInRasterBand->RasterIO(GF_Read, 0, 0, nDemWidth, nDemHeight, pData, nDemWidth, nDemHeight, GDT_Float32, 0, 0);
	if (err == CE_Failure)
	{
		GDALDestroyDriverManager();
		return false;
	}

	//判断图像中是否有异常值，并获取异常值实际值
	float fNoData = 0;
	int nIdx;
	for (int i = 0; i<nDemHeight; i++)
	{
		for (int j = 0; j<nDemWidth; j++)
		{
			nIdx = i*nDemWidth + j;
			if (pData[nIdx] <= -9999)
			{
				fNoData = pData[nIdx];
			}
		}
	}
	//创建矢量图
	const char *pszDriverName = "ESRI Shapefile";

	GDALDriver *poDriver = OGRSFDriverRegistrar::GetRegistrar()->GetDriverByName(pszDriverName);
	GDALAllRegister();
	
	GDALDataset *poDS;

	poDS = poDriver->Create(shpPath, 0, 0, 0, GDT_Unknown, NULL); //创建数据源

	if (poDS == NULL)
	{
		return false;
		exit(1);
	}
	OGRLayer *poLayer;
	OGRSpatialReference *poSpatialRef = new OGRSpatialReference(pInDataset->GetProjectionRef());

	poLayer = poDS->CreateLayer("Elevation", poSpatialRef, wkbLineString, NULL); //创建图层
	if (poLayer == NULL)
	{
		return false;
		exit(1);
	}


	OGRFieldDefn ofieldDef1("Elevation", OFTInteger);    //在矢量图中创建高程值字段
	if (poLayer->CreateField(&ofieldDef1) != OGRERR_NONE)
	{
		GDALClose((GDALDatasetH)poDS);
		GDALClose(pInDataset);
		return false;
	}

	//根据图像波段生成矢量图等高线
	if (fNoData == 0)
		GDALContourGenerate(pInRasterBand, dInterval, 0, 0, NULL, false, 0, (OGRLayerH)poLayer, -1, 0, NULL, NULL);
	else //有异常值时，不对异常值进行处理
		GDALContourGenerate(pInRasterBand, dInterval, 0, 0, NULL, true, fNoData, (OGRLayerH)poLayer, -1, 0, NULL, NULL);

	int count = poLayer->GetFeatureCount();
	OGRFeature *ogrfeature = NULL;
	for (int i = 0; i < count; i++)
	{
		ogrfeature = poLayer->GetFeature(i);
		elevation.push_back(ogrfeature->GetFieldAsInteger(0));

	}
	OGRFeature::DestroyFeature(ogrfeature);

	GDALClose(pInDataset);
	GDALClose(poDS);
	return true;
}