#ifndef _PLANE_H_
#define _PLANE_H_

#include <vector>
#include "POINT3D.h"
#include "myclass.h"
using namespace std;

typedef class PLANE //typedef--用于将Point3D定义为数据类型
{

public:
	PLANE();
	~PLANE();


public:


void roofpoint_facadepoint_m_volume_knn(vector < POINT3D >& incloud,
	int k, double angle_threshold, vector < POINT3D >& roofpoint, vector <  POINT3D >& facadepoint,
	vector < POINT3D >& t_line, vector < POINT3D >& t_plane, vector < POINT3D >& t_volume);
void Hierarchical_objectsegmentation(vector < POINT3D >& incloud, vector < vector < POINT3D >>& roadlinepoint);

void Road_boundary_line_extraction(vector < POINT3D >& incloud, int k, double angle_threshold,
	double Cylinder_radius, double altitude_difference, vector < POINT3D >& boundary_point, double T_plane_recognition);


void Octree_based_regiongrowing_planesegmentation_roof(vector < POINT3D >& incloud, int max_number, int min_number, int Neighbours,
	int k, double SmoothnessThreshold, double CurvatureThreshold, vector<vector<POINT3D>>& roofpoint, double t_angle, double scale, double point_dis);



}plane;