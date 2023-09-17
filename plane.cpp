


#include "stdafx.h"
#include "PLANE.h"
#include "POINT3D.h"
#include "max_min.h"
#include "MBR.h"
#include "myclass.h"
#include "ReadWrite.h"
//#include <pcl/point_cloud.h>        //�����Ͷ���ͷ�ļ�
//#include <pcl/kdtree/kdtree_flann.h> //kdtree�ඨ��ͷ�ļ�
//#include<pcl/kdtree/io.h>
#include<iostream>
#include<vector>
#include<algorithm>
//#include <ros/ros.h>
//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/common/common.h>
#include "ReadWrite.h"
//#include "ORSA_SV.h"
#include "Roof_extraction.h"
#include "projection.h"
#include "Grid.h"
#include "Eight_neighborhood.h"
#include "Boudary.h"
#include "Grid.h"
#include"common.h"
#include"CFSFDP.h"
#include"PCA1.h"
#include <math.h>
//#include <pcl/octree/octree.h>
#include <set>

#define DEBUG
#define RELEASE
using namespace std;








#define PI 3.141592653
PLANE::PLANE() :A(0), B(0), C(0), D(0), curvature(0)
{
}


PLANE::~PLANE()
{
}











////������˵�����ɷ��ݶ����ƣ��ݶ��ͷ�ƽ��㣩������������
void PLANE::roofpoint_facadepoint_m_volume_knn(vector < POINT3D >& incloud,
	int k, double angle_threshold, vector < POINT3D >& roofpoint, vector <  POINT3D >& facadepoint,
	vector < POINT3D >& t_line, vector < POINT3D >& t_plane, vector < POINT3D >& t_volume)
{
	PLANE plane;
	POINT3D point3d;
	Boudary bd;
	CFSFDP cf;
	ReadWrite rw;
	myclass mc;
	PLANE NormalB;
	NormalB.A = 0;
	NormalB.B = 0;
	NormalB.C = 1;
	PLANE NormalA;
	//noplanepoint = incloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
	mc.SwitchDataAsciiToPCL(incloud, pointcloud);
	pcl::KdTreeFLANN<pcl::PointXYZ>kdtree;
	kdtree.setInputCloud(pointcloud);
	//vector<POINT3D>t_line; vector<POINT3D>t_plane; vector<POINT3D>t_volume;
	for (int i = 0; i < incloud.size(); ++i)
	{
		pcl::PointXYZ searchPoint;
		//int K = 10;
		int K = k;
		std::vector<int> pointIdxNKNSearch(K);      //�洢��ѯ���������
		std::vector<float> pointNKNSquaredDistance(K); //�洢���ڵ��Ӧ����ƽ��
		//std::vector<int> pointIdxRadiusSearch;
		//std::vector<float> pointRadiusSquaredDistance;
		//vector<vector<float>> distance;
	//	vector <int>index;
		searchPoint.x = incloud[i].x; //centre.x;
		searchPoint.y = incloud[i].y;
		searchPoint.z = incloud[i].z;
		// �ڰ뾶r����������
		//std::vector<int> pointIdxRadiusSearch;
		//std::vector<float> pointRadiusSquaredDistance;
		kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
		//int a = pointIdxNKNSearch[0];
		//inindices.push_back(a);//���Ի��������ͨ��������÷����������Ը��õĻ����Щ��
		vector < POINT3D > temp_point;
		for (int j = 0; j < K; j++)
		{
			int a = pointIdxNKNSearch[j];
			point3d.x = incloud[a].x;
			point3d.y = incloud[a].y;
			point3d.z = incloud[a].z;
			temp_point.push_back(point3d);
		}
		vector<POINT3D>line; vector<POINT3D>plane; vector<POINT3D>volume;
		bd.PCA_linear_planar_volumetric(temp_point, line, plane, volume);//�ߡ��������ж�
		if (plane.size() > 0)
		{
			bd.PCA_plane_normal(temp_point, NormalA);//���㷨�����ж������滹���ݶ�
			//double angle = plane.PlaneNormalsAngle3D(NormalA, NormalB);
			double angle = PlaneNormalsAngle3D(NormalA, NormalB);//�������¼���
			//m_angle.push_back(angle);
			//vector < POINT3D > temp_facadepoint;
			if (angle > 90)
			{
				if ((angle - 90) > angle_threshold)
				{
					point3d.x = incloud[i].x;
					point3d.y = incloud[i].y;
					point3d.z = incloud[i].z;
					roofpoint.push_back(point3d);

				}
				else
				{
					point3d.x = incloud[i].x;
					point3d.y = incloud[i].y;
					point3d.z = incloud[i].z;
					facadepoint.push_back(point3d);

				}
			}
			else
			{
				if ((90 - angle) > angle_threshold)
				{
					point3d.x = incloud[i].x;
					point3d.y = incloud[i].y;
					point3d.z = incloud[i].z;
					roofpoint.push_back(point3d);

				}
				else
				{
					point3d.x = incloud[i].x;
					point3d.y = incloud[i].y;
					point3d.z = incloud[i].z;
					facadepoint.push_back(point3d);
				}
			}
			point3d.x = incloud[i].x;
			point3d.y = incloud[i].y;
			point3d.z = incloud[i].z;
			t_plane.push_back(point3d);
		}

		if (line.size() > 0)
		{
			point3d.x = incloud[i].x;
			point3d.y = incloud[i].y;
			point3d.z = incloud[i].z;
			roofpoint.push_back(point3d);
			t_line.push_back(point3d);
		}

		if (volume.size() > 0)
		{
			point3d.x = incloud[i].x;
			point3d.y = incloud[i].y;
			point3d.z = incloud[i].z;
			roofpoint.push_back(point3d);
			t_volume.push_back(point3d);
		}
	}
	
}







//�ֲ����ָ��㷨
void PLANE::Hierarchical_objectsegmentation(vector < POINT3D >& incloud, vector < vector < POINT3D >>& roadlinepoint)
{
	//////////
	//��һ�����ܶȾ��ࣻ�ڶ�����ƽ��ָ���������߷ָ���Ĳ�����״��ָ�
	CFSFDP cf;
	ReadWrite rw;
	POINT3D point3d;
	Boudary bd;
	PCA1 pca;
	PLANE NormalB;
	NormalB.A = 0;
	NormalB.B = 0;
	NormalB.C = 1;
	PLANE NormalA;

	
	double object_radius = 0.5;// 0.8;// 0.3;
	int radius_pointnum = 5;
	double angle_threshold = 20;
	int start = clock();
	vector<double>t_angle;


	vector<vector<POINT3D>>discrete_point;

	vector<POINT3D> road_point;
	
	vector<vector<POINT3D>>Roof_Superpoint;
	Roof_Superpoint.push_back(incloud);
	//�˲������������㷨
	//Octree_based_regiongrowing_planesegmentation
	vector<vector<POINT3D>> planepoint;
	vector < POINT3D > noplanepoint;
	vector<vector<POINT3D>> total_planepoint;
	vector<vector<POINT3D>> total_noplanepoint;
	for (int i = 0; i < Roof_Superpoint.size(); i++)
	{
		if (Roof_Superpoint[i].size() > 20)
		{
			vector<vector<POINT3D>> temp_planepoint;
			vector < POINT3D > temp_noplanepoint;
		

			Octree_based_regiongrowing_planesegmentation(Roof_Superpoint[i], 10000000, 25, 25, 25, object_radius, 4.0, 10.0, temp_planepoint, noplanepoint);//�ܺ�
			
			

			for (int j = 0; j < temp_planepoint.size(); j++)
			{
				planepoint.push_back(temp_planepoint[j]);
			}
			

			for (int j = 0; j < planepoint.size(); j++)
			{
				bd.PCA_plane_normal(planepoint[j], NormalA);//���㷨�����ж������滹���ݶ�
				//double angle = plane.PlaneNormalsAngle3D(NormalA, NormalB);
				double angle = PlaneNormalsAngle3D(NormalA, NormalB);//�������¼���
				t_angle.push_back(angle);
				if (angle > 90)
				{
					if ((180 - angle) < angle_threshold)
					{
						for (int k = 0; k < planepoint[j].size(); k++)
						{
							point3d.x = planepoint[j][k].x;
							point3d.y = planepoint[j][k].y;
							point3d.z = planepoint[j][k].z;
							road_point.push_back(point3d);
						}

					}
				}
				else
				{
					if (angle < angle_threshold)
					{
						for (int k = 0; k < planepoint[j].size(); k++)
						{
							point3d.x = planepoint[j][k].x;
							point3d.y = planepoint[j][k].y;
							point3d.z = planepoint[j][k].z;
							road_point.push_back(point3d);
						}
					}
				}
			}


			for (int j = 0; j < planepoint.size(); j++)
			{
				vector < POINT3D >temp_planepoint;
				for (int k = 0; k < planepoint[j].size(); k++)
				{
					point3d.x = planepoint[j][k].x;
					point3d.y = planepoint[j][k].y;
					point3d.z = planepoint[j][k].z;// *shrink;
					temp_planepoint.push_back(point3d);
				}
				total_planepoint.push_back(temp_planepoint);
			}
			planepoint.clear();
			if (noplanepoint.size() > 0)
			{
				//std::cout << noplanepoint.size() << std::endl;
				vector<vector<POINT3D>>noplanepoint_Superpoint = cf.superpoint_segmentation(noplanepoint, object_radius, radius_pointnum);//ANH3
				//std::cout << "over" << std::endl;
				noplanepoint.clear();

				for (size_t j = 0; j < noplanepoint_Superpoint.size(); j++)
				{
					vector < POINT3D >temp_noplanepoint;
					for (int k = 0; k < noplanepoint_Superpoint[j].size(); k++)
					{
						point3d.x = noplanepoint_Superpoint[j][k].x;
						point3d.y = noplanepoint_Superpoint[j][k].y;
						point3d.z = noplanepoint_Superpoint[j][k].z;// *shrink;
						temp_noplanepoint.push_back(point3d);
					}
					total_noplanepoint.push_back(temp_noplanepoint);
				}
			}

		}
		else
		{
			discrete_point.push_back(Roof_Superpoint[i]);
		}

	}
	vector < POINT3D >in_out_noplanepoint;
	for (int i = 0; i < total_noplanepoint.size(); i++)
	{
		vector < POINT3D >temp_noplanepoint;
		for (int j = 0; j < total_noplanepoint[i].size(); j++)
		{
			point3d.x = total_noplanepoint[i][j].x;
			point3d.y = total_noplanepoint[i][j].y;
			point3d.z = total_noplanepoint[i][j].z;// *shrink;
			in_out_noplanepoint.push_back(point3d);
		}

	}

	string outtxt_txtdiscrete_point = "E:\\discrete_point.txt";
	//����ά���ݱ��浥�廯��ɫ�ĵ���������
	vector<POINT3D>m_IndividualRGBdiscrete_point = rw.IndividualRGBpointcloud(discrete_point);
	rw.write_txt(outtxt_txtdiscrete_point, m_IndividualRGBdiscrete_point);


	string outtxt_txttotal_planepoint = "E:\\total_planepoint.txt";
	//����ά���ݱ��浥�廯��ɫ�ĵ���������
	vector<POINT3D>m_IndividualRGBtotal_planepoint = rw.IndividualRGBpointcloud(total_planepoint);
	rw.write_txt(outtxt_txttotal_planepoint, m_IndividualRGBtotal_planepoint);

	string outtxt_txttotal_noplanepoint = "E:\\total_noplanepoint.txt";
	//����ά���ݱ��浥�廯��ɫ�ĵ���������
	vector<POINT3D>m_IndividualRGBtotal_noplanepoint = rw.IndividualRGBpointcloud(total_noplanepoint);
	rw.write_txt(outtxt_txttotal_noplanepoint, m_IndividualRGBtotal_noplanepoint);
	cout << "ƽ��ͷ�ƽ���  :" << (double)(clock() - start) / CLOCKS_PER_SEC << endl;

	
	//double roadline_radius2d = 0.3;
	//double T_high_difference = 0.3;
	double roadline_radius2d = 0.2;
	double T_high_difference = 0.2;
	vector < POINT3D >boundarylinepoint;//�������ı߽���

	bd.Judgment_of_road_boundary_line_with_height_difference(in_out_noplanepoint, roadline_radius2d, T_high_difference, boundarylinepoint);

	string outtxt_txtboundarylinepoint = "E:\\boundarylinepoint.txt";
	////����ά���ݱ��浥�廯��ɫ�ĵ���������
	//vector<POINT3D>m_IndividualRGBboundarylinepoint = rw.IndividualRGBpointcloud(boundarylinepoint);
	rw.write_txt(outtxt_txtboundarylinepoint, boundarylinepoint);
	cout << "����ȡ��·�߽���  :" << (double)(clock() - start) / CLOCKS_PER_SEC << endl;


	double radius3d01 = 0.8;
	double Average_points01;
	double neighbor_pointnum01 = 100;
	vector < POINT3D > outcloud01;
	bd.pointclouddensity_inthelocalneighborhood(boundarylinepoint, radius3d01, Average_points01, neighbor_pointnum01, outcloud01);
	string outtxt_txtoutcloud01 = "E:\\outcloud01.txt";
	////����ά���ݱ��浥�廯��ɫ�ĵ���������
	//vector<POINT3D>m_IndividualRGBboundarylinepoint = rw.IndividualRGBpointcloud(boundarylinepoint);
	rw.write_txt(outtxt_txtoutcloud01, outcloud01);


	//double radius2d=0.5;
	//double cluster_radius=0.55;
	//double line_judge1=3; 
	//double line_judge2 = 6;
	//double T_no_ground2=0.3;//��·�߽��������ĸ߲���ڸ�ֵΪ�ǵ�·�߽��ߵ�
	double radius2d = 0.5;
	//double cluster_radius = 0.8;
	double cluster_radius = 0.6;
	double line_judge1 = 3;
	double line_judge2 = 6;
	double T_no_ground2 = 0.25;//��·�߽��������ĸ߲���ڸ�ֵΪ�ǵ�·�߽��ߵ�




	vector < POINT3D > ture_roadlinepoint;
	//bd.Delete_non_road_boundary_points(boundarylinepoint, radius2d, cluster_radius, line_judge1, line_judge2, T_no_ground2, delete_noground_outcloud, ture_roadlinepoint);
	bd.Delete_non_road_boundary_points(boundarylinepoint, radius2d, cluster_radius, line_judge1, line_judge2, T_no_ground2, road_point, ture_roadlinepoint);

	string outtxt_txtture_roadlinepoint = "E:\\ture_roadlinepoint.txt";
	////����ά���ݱ��浥�廯��ɫ�ĵ���������
	//vector<POINT3D>m_IndividualRGBboundarylinepoint = rw.IndividualRGBpointcloud(boundarylinepoint);
	rw.write_txt(outtxt_txtture_roadlinepoint, ture_roadlinepoint);

	double radius3d = 0.6;
	double Average_points;
	double neighbor_pointnum = 50;
	vector < POINT3D > outcloud;

	//double radius3d = 0.5;
	//double Average_points;
	//double neighbor_pointnum = 30;
	//vector < POINT3D > outcloud;

	bd.pointclouddensity_inthelocalneighborhood(ture_roadlinepoint, radius3d, Average_points, neighbor_pointnum, outcloud);
	string outtxt_txtoutcloud = "E:\\outcloud.txt";
	////����ά���ݱ��浥�廯��ɫ�ĵ���������
	//vector<POINT3D>m_IndividualRGBboundarylinepoint = rw.IndividualRGBpointcloud(boundarylinepoint);
	rw.write_txt(outtxt_txtoutcloud, outcloud);
	vector<vector<POINT3D>>Superpoint = cf.superpoint_segmentation(outcloud, 0.85, 0);//ANH3
	vector<vector<POINT3D>>afor_ture_roadlinepoint;
	double length = 0;  double width = 0;  double angle = 0;
	for (int i = 0; i < Superpoint.size(); i++)
	{
		pca.PCAalgorithm(Superpoint[i], length, width, angle);
		if ((length > 2.0) || (width > 2.0))
		{
			afor_ture_roadlinepoint.push_back(Superpoint[i]);
		}
	}
	string outtxt_txtafor_ture_roadlinepoint = "E:\\afor_ture_roadlinepoint.txt";
	////����ά���ݱ��浥�廯��ɫ�ĵ���������
	vector<POINT3D>m_IndividualRGBboundaryafor_ture_roadlinepoint = rw.IndividualRGBpointcloud(afor_ture_roadlinepoint);
	rw.write_txt(outtxt_txtafor_ture_roadlinepoint, m_IndividualRGBboundaryafor_ture_roadlinepoint);





	double radius3d1 = 0.6;
	//double Average_points;
	//double neighbor_pointnum = 80;
	double cluster_radius1 = 0.6;
	int localneighbour_num = 70;
	double localneighbour_num_rata = 0.850;


	bd.Pointcloudcluster_density_andPCA_extractroadboundaryline(ture_roadlinepoint, radius3d1, cluster_radius1, localneighbour_num,
		localneighbour_num_rata, roadlinepoint);



	cout << "����ʱ��  :" << (double)(clock() - start) / CLOCKS_PER_SEC << endl;
}

//��·�߽�����ȡʧ�ܣ��������ڶ�߶�pca������Ƶ����ԡ���״����״��Ϣ
void PLANE::Road_boundary_line_extraction(vector < POINT3D >& incloud, int k, double angle_threshold,
	double Cylinder_radius, double altitude_difference, vector < POINT3D >& boundary_point, double T_plane_recognition)
{

	PLANE plane;
	POINT3D point3d;
	Boudary bd;
	CFSFDP cf;
	ReadWrite rw;
	myclass mc;
	PLANE NormalB;
	NormalB.A = 0;
	NormalB.B = 0;
	NormalB.C = 1;
	PLANE NormalA;
	vector < POINT3D >temp_boundary_point;


	//noplanepoint = incloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
	mc.SwitchDataAsciiToPCL(incloud, pointcloud);
	pcl::KdTreeFLANN<pcl::PointXYZ>kdtree;
	kdtree.setInputCloud(pointcloud);
	//vector<POINT3D>t_line; vector<POINT3D>t_plane; vector<POINT3D>t_volume;

	for (int i = 0; i < incloud.size(); i++)
	{
		int K = k;
		for (; K < 600; K = K + 50)
		{
			pcl::PointXYZ searchPoint;
			//int K = 10;
			//int K = k;
			std::vector<int> pointIdxNKNSearch(K);      //�洢��ѯ���������
			std::vector<float> pointNKNSquaredDistance(K); //�洢���ڵ��Ӧ����ƽ��
			//std::vector<int> pointIdxRadiusSearch;
			//std::vector<float> pointRadiusSquaredDistance;
			//vector<vector<float>> distance;
		//	vector <int>index;
			searchPoint.x = incloud[i].x; //centre.x;
			searchPoint.y = incloud[i].y;
			searchPoint.z = incloud[i].z;
			// �ڰ뾶r����������
			//std::vector<int> pointIdxRadiusSearch;
			//std::vector<float> pointRadiusSquaredDistance;
			kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
			//int a = pointIdxNKNSearch[0];
			//inindices.push_back(a);//���Ի��������ͨ��������÷����������Ը��õĻ����Щ��
			vector < POINT3D > temp_point;
			for (int j = 0; j < K; j++)
			{
				int a = pointIdxNKNSearch[j];
				point3d.x = incloud[a].x;
				point3d.y = incloud[a].y;
				point3d.z = incloud[a].z;
				temp_point.push_back(point3d);
			}
			vector<POINT3D>line; vector<POINT3D>plane; vector<POINT3D>volume;
			double a1d = 0; double a2d = 0; double a3d = 0;
			//bd.Multiscale_PCA_linear_planar_volumetric(temp_point, a1d, a2d, a3d);//�ߡ��������ж�
			bd.PCA_linear_planar_volumetric(temp_point, line, plane, volume);//�ߡ��������ж�
			double plane_recognition1 = a1d / a3d;
			double plane_recognition2 = a2d / a3d;


			if (plane.size() > 0)
			{
				point3d.x = incloud[i].x;
				point3d.y = incloud[i].y;
				point3d.z = incloud[i].z;
				temp_boundary_point.push_back(point3d);
			}


			
		}


	}

	string outtxt_t_temp_boundary_point = "E:\\t_temp_boundary_point.txt";
	//vector<POINT3D>m_t_volume_Superpoint = rw.IndividualRGBpointcloud(t_volume_Superpoint);
	rw.write_txt(outtxt_t_temp_boundary_point, temp_boundary_point);



	//��������ÿ����Ϊ���ģ��жϸõ������������뾶����͵�ĸ߲�С����ֵ��Ϊ��·�߽��ߣ�


}
















////������˵����Octree-based region growing for point cloud segmentation
////������˵�������ڰ˽��Ƶ����������Խ��е��Ʒָ�(ȥ��������ƣ�����ݶ�����)
////�����������incloud------------------�����������
////�����������max_number---------ÿ��ƽ����ຬ�ж��ٸ���
////�����������min_number---------ÿ��ƽ�����ٺ��ж��ٸ���
////�����������Neighbours----------�ο��ڽӵ����
////�����������k-----------����������������ķ�����
////�����������SmoothnessThreshold-----------------����ά����ά���߶Ƚ�����[11]
////�����������CurvatureThreshold-----------��ά�߶Ƚ�����С��ࣨѭ����ʱ��0.8������ã�[10]
////�����������planepoint-----------------����ά����ά���߶Ƚ�����[11]noplanepoint
////�����������noplanepoint-----------------����ά����ά���߶Ƚ�����[11]
void PLANE::Octree_based_regiongrowing_planesegmentation_roof(vector < POINT3D >& incloud, int max_number, int min_number, int Neighbours,
	int k, double SmoothnessThreshold, double CurvatureThreshold, vector<vector<POINT3D>>& roofpoint, double t_angle, double scale, double point_dis)//vector < POINT3D > &noplanepoint
{
	myclass mc;
	POINT3D point3d;
	max_min mm;
	PLANE plane;
	PLANE NormalB;
	PLANE NormalA;
	POINT3D max, min;
	Boudary bd;
	vector < POINT3D > icloud;
	mm.max_min_calculation(incloud, max, min);
	for (int i = 0; i < incloud.size(); i++)
	{
		point3d.x = incloud[i].x - min.x;
		point3d.y = incloud[i].y - min.y;
		point3d.z = incloud[i].z - min.z;
		icloud.push_back(point3d);
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	mc.SwitchDataAsciiToPCL(icloud, cloud);
	pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	//���������**********************************************
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(cloud);
	normal_estimator.setKSearch(k);  //k
	normal_estimator.compute(*normals);
	//���ڷ����������ʵ����������㷨**************************
	pcl::IndicesPtr indices(new std::vector <int>);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 1.0);
	pass.filter(*indices);

	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize(min_number);   //min_number
	reg.setMaxClusterSize(max_number);		//���Ƽ�������max_number
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(Neighbours);		//�ο��ڽӵ����Neighbours
	reg.setInputCloud(cloud);

	//reg.setIndices (indices);

	reg.setInputNormals(normals);
	double Smoothness = SmoothnessThreshold / 180.0 * M_PI;
	reg.setSmoothnessThreshold(Smoothness);		//ƽ����ֵSmoothnessThreshold
	reg.setCurvatureThreshold(CurvatureThreshold);		//������ֵCurvatureThreshold

	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters);
	vector < vector<POINT3D>>facade_point;
	vector<vector<POINT3D>>planepoint;
	int b = 0;
	for (int i = 0; i < clusters.size(); i++)
	{
		vector<POINT3D>temp_planepoint;
		for (int j = 0; j < clusters[i].indices.size(); j++)
		{
			b = clusters[i].indices[j];
			point3d.x = icloud[b].x + min.x;
			point3d.y = icloud[b].y + min.y;
			point3d.z = icloud[b].z + min.z;
			temp_planepoint.push_back(point3d);
		}
		planepoint.push_back(temp_planepoint);
		temp_planepoint.clear();
		vector<POINT3D>().swap(temp_planepoint);
	}
	for (int i = 0; i < planepoint.size(); i++)
	{
		vector<int> p_inliersindex;
		vector<POINT3D> p_inlierpoints;
		vector<planar_3d> p_planar_coefficients;
		bd.PCA_plane_normal(planepoint[i], NormalA);
		double angle = plane.PlaneNormalsAngle3D(NormalA, NormalB);//�������¼���
		//NormalA = plane.NEWLeastSquaresPlaneFitting(planepoint[i]);
		//mc.my_single_plane_pclransac(planepoint[i], 0.5, p_inliersindex, p_inlierpoints, p_planar_coefficients);
		/*NormalA.A = p_planar_coefficients[0].A;
		NormalA.B = p_planar_coefficients[0].B;
		NormalA.C = p_planar_coefficients[0].C;*/
		//double angle = plane.PlaneNormalsAngle3D(NormalA, NormalB);//�������¼���
		//m_angle.push_back(angle);
		double angle1 = abs(angle);

		if (angle1 > 90)
		{
			if ((angle1 - 90) < t_angle)
			{
				facade_point.push_back(planepoint[i]);
			}
		}
		else
		{
			if ((90 - angle1) < t_angle)
			{
				facade_point.push_back(planepoint[i]);
			}
		}
	}

	for (int i = 0; i < facade_point.size(); i++)
	{
		for (int j = 0; j < facade_point[i].size(); j++)
		{
			int k = 1;
			pcl::PointXYZ searchpoint;
			searchpoint.x = facade_point[i][j].x;
			searchpoint.x = facade_point[i][j].x;
			searchpoint.x = facade_point[i][j].x;
			std::vector<int>pointidnknsearch(k);
			std::vector<float>pointidnknsearchDistance(k);
			if (tree->nearestKSearch(searchpoint, k, pointidnknsearch, pointidnknsearchDistance))
			{
				int a = pointidnknsearch[0];
				incloud[a].IsVisisted = true;
			}
		}
	}
	vector<POINT3D>out_cloud_enlarge;
	for (int i = 0; i < incloud.size(); i++)
	{
		if (incloud[i].IsVisisted == false)
		{
			point3d.x = incloud[i].x;
			point3d.y = incloud[i].y;
			point3d.z = incloud[i].z * scale;
			out_cloud_enlarge.push_back(point3d);
		}
	}
	double cluster_tolerance = point_dis * scale / 2;
	vector<vector<int>>index;
	vector<vector<POINT3D>> out_cloud = mc.OnEuclideanClusterExtraction3D(out_cloud_enlarge, min_number, max_number, cluster_tolerance, index);
	index.clear();
	vector<vector<int>>().swap(index);
	for (int i = 0; i < out_cloud.size(); i++)//roofpoint
	{
		vector<POINT3D>temp_point;
		for (int j = 0; j < out_cloud[i].size(); j++)
		{
			point3d.x = out_cloud[i][j].x;
			point3d.y = out_cloud[i][j].y;
			point3d.z = out_cloud[i][j].z / scale;
			temp_point.push_back(point3d);
		}
		roofpoint.push_back(temp_point);
	}




	/*string outtxt_txt01out_cloud = "E:\\out_cloud.txt";
	vector<POINT3D>m_IndividualRGBout_cloud = rw.IndividualRGBpointcloud(out_cloud);
	rw.write_txt(outtxt_txt01out_cloud, m_IndividualRGBout_cloud);
	string outtxt_txt01planepoint = "E:\\planepoint.txt";
	vector<POINT3D>m_IndividualRGBplanepoint = rw.IndividualRGBpointcloud(planepoint);
	rw.write_txt(outtxt_txt01planepoint, m_IndividualRGBplanepoint);
	string outtxt_txt01noplanepoint = "E:\\noplanepoint.txt";
	rw.write_txt(outtxt_txt01noplanepoint, noplanepoint);
*/


//int k = 0;
//for (int i = 0; i < clusters.size(); i++)
//{
//	k = k + clusters[i].indices.size();
//	/*for (int j=0;j< clusters[i].indices.size();j++)
//	{

//	}*/
//}

//vector<int>clusters_index;
//for (int i = 0; i < clusters.size(); i++)
//{
//	vector<int>temp_clusters_index;
//	for (int j = 0; j < clusters[i].indices.size(); j++)
//	{
//		temp_clusters_index.push_back(clusters[i].indices[j]);
//	}
//	for (int j = 0; j < temp_clusters_index.size(); j++)
//	{
//		clusters_index.push_back(temp_clusters_index[j]);
//	}
//	temp_clusters_index.clear();
//	vector<int>().swap(temp_clusters_index);
//}
//sort(clusters_index.begin(), clusters_index.end());
//int kk = 0;
///*vector<int> inner_index;
//vector<int> out_index;*/

//for (int i = 0; i < icloud.size(); i++)
//{
//	int a = clusters_index[kk];
//	if (i != a)
//	{
//		point3d.x = icloud[a].x + min.x;
//		point3d.y = icloud[a].y + min.y;
//		point3d.z = icloud[a].z + min.z;
//		noplanepoint.push_back(point3d);
//	}
//	else
//	{
//		kk++;
//	}
//}

//�������ӣ�https://blog.csdn.net/weixin_42795525/article/details/99653785
}














