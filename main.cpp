#include "stdafx.h"
#define BOOST_TYPEOF_EMULATION
#include "Boudary.h"
#include "CFSFDP.h"
//#include "ClusterAnalysis.h"
#include "Eight_neighborhood.h"
#include "Grid.h"
#include "lasreader.hpp"
#include "laswriter.hpp"
#include <iostream>
#include "isodata.h"
#include "image.h"
#include "max_min.h"
#include "myclass.h"
#include "myfunction.h"
#include "MBR.h"
#include "PLANE.h"
#include "PCA1.h"
#include "ReadWrite.h"
#include "Roof_extraction.h"
#include <fstream>
#include <string>
#include<stdlib.h>
#include"typedef.h"
#include "time.h"
#include <vector>
using namespace std;
typedef std::string String;
#define Random(x) (rand() % x)


int main(int argc, char** argv)
{
	//ClusterAnalysis cla;
	int start = clock();
	///加载点云数据
	myclass mc;
	ReadWrite rw;
	MBR mbr;
	max_min mm;
	Roof_extraction re;
	myfunction my;
	Grid grid;
	CFSFDP cf;
	Eight_neighborhood en;
	image ig;
	Boudary bd;
	PCA1 pca;
	CFSFDP cp;
	PLANE plane;
	PLANE NormalB;
	NormalB.A = 0;
	NormalB.B = 0;
	NormalB.C = 1;
	PLANE NormalA;
	Point3D point3d;
	const char* read_txt_path = argv[1]; //"E:/tsk.tsk";
	//tsk文件的读取
	int c = rw.read_tsk(read_txt_path);
	if (c == -1)//异常判断，确定该tsk是否正常打开
	{
		return 0;
	}
	vector<double >a;
	for (int i = 3; i < rw.m_whole_tsk_file_path.size(); i++)
	{
		double b = atof(rw.m_whole_tsk_file_path[i].c_str());
		a.push_back(b);
	}
	int a2 = atof(rw.m_whole_tsk_file_path[2].c_str());
	int a3 = atof(rw.m_whole_tsk_file_path[3].c_str());

	if (rw.m_whole_tsk_file_path.size() > 0)
	{
		//////读
#pragma region	


		vector<POINT3D>ground2;//地面点
		vector<POINT3D>unclassification1;//非地面点
		vector<POINT3D>noise7;//噪声7
		string intxt = rw.m_whole_tsk_file_path[0];
		Point3D point3d;
		vector<POINT3D > icloud;
		//打开las文件
		LASreadOpener lasreadopener;


		vector < vector<POINT3D>>buildingpoint;
		vector < vector<POINT3D>>total_buildingpoint;
		LASreader* lasreader;

		string file_name = rw.m_whole_tsk_file_path[0];
		const char* file_name_txt = file_name.c_str();
		vector<POINT3D>temp_buildingpoint;
		vector<POINT3D>temp_total_buildingpoint;

		//readtxt(file_name_txt, temp_point);
		//incloud.push_back(temp_point);
		//temp_point.clear();

		POINT3D maxpoint, minpoint;
		POINT3D t_maxpoint, t_minpoint;
		lasreadopener.set_file_name(file_name.c_str());
		//LASreader* lasreader = lasreadopener.open();
		lasreader = lasreadopener.open();
		lasreader->header;
		if (lasreader == NULL)
		{
			printf("Can't open the las file !");
			return 0;
		}
		//int ii = 10000000;
		//int jj = 20000000;
		while (lasreader->read_point())
		{
			////ii++;
			//if (temp_total_buildingpoint.size() ==ii )
			//{
			//	temp_total_buildingpoint.clear();
			//	vector<POINT3D>().swap(temp_total_buildingpoint);
			//}
			//if (temp_total_buildingpoint.size()==jj)
			//{
			//	break;
			//}
			//laswriter->write_point(&lasreader->point);
			point3d.x = lasreader->point.X * lasreader->header.x_scale_factor + lasreader->header.x_offset;
			point3d.y = lasreader->point.Y * lasreader->header.y_scale_factor + lasreader->header.y_offset;
			point3d.z = lasreader->point.Z * lasreader->header.z_scale_factor + lasreader->header.z_offset;

			if ((lasreader->point.rgb[0] & 0xFF00) || (lasreader->point.rgb[1] & 0xFF00) || (lasreader->point.rgb[2] & 0xFF00))//如果是16进制就进行转换
			{
				point3d.r = lasreader->point.rgb[0] >> 8;
				point3d.g = lasreader->point.rgb[1] >> 8;
				point3d.b = lasreader->point.rgb[2] >> 8;
			}
			else//否则就不进行转换
			{
				point3d.r = lasreader->point.rgb[0];
				point3d.g = lasreader->point.rgb[1];
				point3d.b = lasreader->point.rgb[2];

			}
			/*point3d.r = lasreader->point.rgb[0] >> 8;
			point3d.g = lasreader->point.rgb[1] >> 8;
			point3d.b = lasreader->point.rgb[2] >> 8;*/

			/*point3d.r = lasreader->point.rgb[0];
			point3d.g = lasreader->point.rgb[1];
			point3d.b = lasreader->point.rgb[2];*/
			point3d.intens = lasreader->point.intensity;
			point3d.Classification = lasreader->point.classification;
			temp_total_buildingpoint.push_back(point3d);

			//直接保存建筑物点云
			if ((lasreader->point.classification == a2) || (lasreader->point.classification == a3))
				//if (lasreader->point.classification == a2)// || (lasreader->point.classification == a3))
			{
				//laswriter->write_point(&lasreader->point);
				point3d.x = lasreader->point.X * lasreader->header.x_scale_factor + lasreader->header.x_offset;
				point3d.y = lasreader->point.Y * lasreader->header.y_scale_factor + lasreader->header.y_offset;
				point3d.z = lasreader->point.Z * lasreader->header.z_scale_factor + lasreader->header.z_offset;
				if ((lasreader->point.rgb[0] & 0xFF00) || (lasreader->point.rgb[1] & 0xFF00) || (lasreader->point.rgb[2] & 0xFF00))//如果是16进制就进行转换
				{
					point3d.r = lasreader->point.rgb[0] >> 8;
					point3d.g = lasreader->point.rgb[1] >> 8;
					point3d.b = lasreader->point.rgb[2] >> 8;
				}
				else//否则就不进行转换
				{
					point3d.r = lasreader->point.rgb[0];
					point3d.g = lasreader->point.rgb[1];
					point3d.b = lasreader->point.rgb[2];
				}
				point3d.intens = lasreader->point.intensity;
				point3d.Classification = lasreader->point.classification;
				temp_buildingpoint.push_back(point3d);


			}
			//}
			/*else
			{
				break;
			}*/
		}
		if (temp_total_buildingpoint.size() > 0)
		{
			total_buildingpoint.push_back(temp_total_buildingpoint);
		}
		if (temp_buildingpoint.size() > 0)
		{
			buildingpoint.push_back(temp_buildingpoint);
		}

		vector<vector<POINT3D>>local_building;
		vector<vector<POINT3D>>local_toatlbuilding;
		/*POINT3D maxpoint, minpoint;
		POINT3D t_maxpoint, t_minpoint;*/
		if (buildingpoint.size() > 0)
		{
			vector<POINT3D > building;
			for (int i = 0; i < buildingpoint.size(); i++)
			{
				for (int j = 0; j < buildingpoint[i].size(); j++)
				{
					point3d.x = buildingpoint[i][j].x;
					point3d.y = buildingpoint[i][j].y;
					point3d.z = buildingpoint[i][j].z;
					building.push_back(point3d);
				}
			}

			if (building.size() > 0)
			{
				mm.max_min_calculation(building, maxpoint, minpoint);
			}

			for (int i = 0; i < buildingpoint.size(); i++)
			{
				vector<POINT3D>temp_point;
				for (int j = 0; j < buildingpoint[i].size(); j++)
				{
					point3d.x = buildingpoint[i][j].x - minpoint.x;
					point3d.y = buildingpoint[i][j].y - minpoint.y;
					point3d.z = buildingpoint[i][j].z - minpoint.z;
					point3d.r = buildingpoint[i][j].r;
					point3d.g = buildingpoint[i][j].g;
					point3d.b = buildingpoint[i][j].b;
					point3d.intens = buildingpoint[i][j].intens;
					point3d.Classification = buildingpoint[i][j].Classification;
					temp_point.push_back(point3d);
				}
				local_building.push_back(temp_point);
			}
		}
		if (total_buildingpoint.size() > 0)
		{
			vector<POINT3D > t_building;
			for (int i = 0; i < total_buildingpoint.size(); i++)
			{
				for (int j = 0; j < total_buildingpoint[i].size(); j++)
				{
					point3d.x = total_buildingpoint[i][j].x;
					point3d.y = total_buildingpoint[i][j].y;
					point3d.z = total_buildingpoint[i][j].z;
					t_building.push_back(point3d);
				}
			}

			if (t_building.size() > 0)
			{
				mm.max_min_calculation(t_building, t_maxpoint, t_minpoint);
			}

			for (int i = 0; i < total_buildingpoint.size(); i++)
			{
				vector<POINT3D>temp_totalpoint;
				for (int j = 0; j < total_buildingpoint[i].size(); j++)
				{
					point3d.x = total_buildingpoint[i][j].x - t_minpoint.x;
					point3d.y = total_buildingpoint[i][j].y - t_minpoint.y;
					point3d.z = total_buildingpoint[i][j].z - t_minpoint.z;
					point3d.r = total_buildingpoint[i][j].r;
					point3d.g = total_buildingpoint[i][j].g;
					point3d.b = total_buildingpoint[i][j].b;
					point3d.intens = total_buildingpoint[i][j].intens;
					point3d.Classification = total_buildingpoint[i][j].Classification;
					temp_totalpoint.push_back(point3d);
				}
				local_toatlbuilding.push_back(temp_totalpoint);
			}
		}



		vector < vector < POINT3D >> roadlinepoint;
		plane.Hierarchical_objectsegmentation(local_toatlbuilding[0], roadlinepoint);
		vector < vector < POINT3D >> total_roadlinepoint;
		for (int i = 0; i < roadlinepoint.size(); i++)
		{
			vector < POINT3D >temp_point;
			for (int j = 0; j < roadlinepoint[i].size(); j++)
			{
				point3d.x = roadlinepoint[i][j].x + minpoint.x;
				point3d.y = roadlinepoint[i][j].y + minpoint.y;
				point3d.z = roadlinepoint[i][j].z + minpoint.z;
				temp_point.push_back(point3d);
			}
			total_roadlinepoint.push_back(temp_point);
		}



		string outtxt_txt_high_big_building = "E:\\total_roadlinepoint.txt";
		//将多维数据保存单体化颜色的点云数据中
		vector<POINT3D>m_IndividualRGBpointcloud_high_big_building = rw.IndividualRGBpointcloud(total_roadlinepoint);
		rw.write_txt(outtxt_txt_high_big_building, m_IndividualRGBpointcloud_high_big_building);











	}




	return 0;



	

	
}