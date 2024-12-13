#pragma once
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
void paint_voronoi(cv::Mat& img, cv::Subdiv2D& subdiv)
{
	vector<vector<cv::Point2f> > facets;
	vector<cv::Vec6f> tris;
	vector<cv::Point2f> centers;
	subdiv.getVoronoiFacetList(vector<int>(), facets, centers);
	subdiv.getTriangleList(tris);
	vector<cv::Point> ifacet;
	vector<vector<cv::Point> > ifacets(1);
	vector<cv::Point> trit;
	vector<vector<cv::Point> > itris(1);

	for (size_t i = 0; i < facets.size(); i++)
	{
		ifacet.resize(facets[i].size());
		for (size_t j = 0; j < facets[i].size(); j++)
			ifacet[j] = facets[i][j];

        cv::Scalar color;
		color[0] = rand() & 255;
		color[1] = rand() & 255;
		color[2] = rand() & 255;
		fillConvexPoly(img, ifacet, color, 8, 0);

		ifacets[0] = ifacet;
		polylines(img, ifacets, true, cv::Scalar(), 1, CV_AA, 0);
		circle(img, centers[i], 3, cv::Scalar(), -1, CV_AA, 0);
	}

	//for (int i = 0; i < 1; i++) {
	//	trit.resize(3);
	//	for (int j = 0; j < 3; j++) {
	//		trit[j].x = tris[i].val[j * 2];
	//		trit[j].y = tris[i].val[j * 2 + 1];
	//	}
	//	itris[0] = trit;
	//	polylines(img, itris, true, cv::Scalar(0, 0, 255), 1, CV_AA, 0);

	//}

	//circle(img, centers[0], 3, cv::Scalar(), -1, CV_AA, 0);
	//circle(img, facets[0][0], 3, cv::Scalar(), -1, CV_AA, 0);
}


static vector<float> paint_voronoi_nopaint(cv::Subdiv2D& subdiv, int* edge_flag)
{   
    *edge_flag = 0;
    vector<vector<cv::Point2f> > facets;
    vector<cv::Vec6f> tris;
    vector<cv::Point2f> centers;
    subdiv.getVoronoiFacetList(vector<int>(), facets, centers);
    subdiv.getTriangleList(tris);
    vector<float> coord(2);
    
    for (int i = 0; i < facets[0].size(); i++) {
        if (facets[0][i].x>=1000 || facets[0][i].x<=0 || facets[0][i].y>=1000 || facets[0][i].y<=0) {
            *edge_flag= 1;
            break;
        }
        coord[0] = coord[0] + facets[0][i].x;
        coord[1] = coord[1] + facets[0][i].y;
    }
    coord[0] = coord[0] / facets[0].size();
    coord[1] = coord[1] / facets[0].size();
    return coord;
}

void voronoi_calculate_nopaint(pcl::PointCloud<pcl::PointXYZ>::Ptr& xoy_points)
{
    cv::Rect rect(0, 0, 1000, 1000);
    //画图需要
	//cv::Mat img(rect.size(), CV_8UC3);
	//img = cv::Scalar::all(0);
	//string win = "Delaunay Demo";

    cv::Subdiv2D subdiv(rect);
    float XmaxCoord, XminCoord, YmaxCoord, YminCoord;
    XmaxCoord = -1e+10;
    XminCoord = 1e+10;
	YmaxCoord = -1e+10;
	YminCoord = 1e+10;
    for (int i = 0; i < xoy_points->points.size(); i++) {
        if (xoy_points->points[i].x > XmaxCoord) {
            XmaxCoord = xoy_points->points[i].x;
        }
        if (xoy_points->points[i].y > YmaxCoord) {
            YmaxCoord = xoy_points->points[i].y;
        }

        if (xoy_points->points[i].x < XminCoord) {
            XminCoord = xoy_points->points[i].x;
        }
        if (xoy_points->points[i].y < YminCoord) {
            YminCoord = xoy_points->points[i].y;
        }
    }
    if (fabs(XmaxCoord-XminCoord)<1e-5 || fabs(YmaxCoord - YminCoord) < 1e-5)
    {
        return;
    }
    vector<vector<float>>xoyNormalized(xoy_points->points.size());
    //范围变成0-1000
	for (int i = 0; i < xoyNormalized.size(); i++) {
       xoyNormalized[i].push_back(((xoy_points->points[i].x - XminCoord) * 999.9 / (XmaxCoord - XminCoord)));
       xoyNormalized[i].push_back(((xoy_points->points[i].y - YminCoord) * 999.9 / (YmaxCoord - YminCoord)));
    }
    //求解质心
    //vector<float>centroidXOZ(2,0);
	//for (int i = 0; i < xoyNormalized.size(); i++) {
 //       centroidXOZ[0] += xoyNormalized[i][0];
 //       centroidXOZ[1] += xoyNormalized[i][1];
	//}
	//centroidXOZ[0] /= xoyNormalized.size();
	//centroidXOZ[1] /= xoyNormalized.size();
 //   //质心变成（500,500）
	//for (int i = 0; i < xoyNormalized.size(); i++) {
 //       xoyNormalized[i][0] = xoyNormalized[i][0] - (centroidXOZ[0] - 500);
 //       xoyNormalized[i][1] = xoyNormalized[i][1] - (centroidXOZ[1] - 500);
	//}
    for (int i = 0; i < xoyNormalized.size(); i++) {
        cv::Point2f fp(xoyNormalized[i][0], xoyNormalized[i][1]);
        subdiv.insert(fp);
    }
    vector<float> coord;
    int edge_flag;

    //画图
	//img = cv::Scalar::all(0);
	//paint_voronoi(img, subdiv);

	//imshow(win, img);

	//cv::waitKey(0);


    coord = paint_voronoi_nopaint(subdiv, &edge_flag);
    
    if (edge_flag == 0) {
        //回归原始倍率
		coord[0] = (coord[0] / 999.9) * (XmaxCoord - XminCoord) + XminCoord;
		coord[1] = (coord[1] / 999.9) * (YmaxCoord - YminCoord) + YminCoord;

        xoy_points->points[0].x  = coord[0];
        xoy_points->points[0].y  = coord[1];
    }
}





