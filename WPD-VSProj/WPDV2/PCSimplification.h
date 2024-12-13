#pragma once
#include "PcLoad.hpp"
#include <pcl/kdtree/kdtree_flann.h>  //kdtree近邻搜索
using namespace std;
class PCSimplification
{
public:
	//构造函数  需要点云文件夹路径、保存文件夹路径
	PCSimplification(string fileBase, string saveBase);
	//点云简化执行
	int doSimplification();
public:
	//设置简化点云目标点数
	void SetResultNumber(int simplification);
	//设置切空间平滑次数，默认为5次
	void SetVoronoiNumber(int voronoiNum);
	//切空间优化邻居个数 (K-1)，默认K=13
	void SetVoronoiKNN(int K);
	//设置是否执行切空间平滑，默认执行
	void SetVoronoi(bool voronoiFlag);
	//设置是否进行特征保持
	void SetEdge(string edgeBase);
private:
	//点云简化点数
	int simplification_{ 10000 };
	//切空间平滑次数，默认5次
	int voronoiNum_{5};
	//切空间优化邻居个数 (K-1)，默认K=13
	int K_{ 13 };
	//是否执行切空间平滑,默认执行
	bool voronoiFlag_{ true };
	//是否执行特征保持，默认不执行
	bool edgeFlag_{ false };
private:
	//点云模型文件夹地址
	string fileBase_;
	//点云边缘文件夹地址
	string edgeBase_;
	//需要保存的文件夹地址
	string saveBase_;
	//误差评价需要保存的文件夹地址
	string errorSaveBase_;
	//点云模型文件地址
	string filePath_;
	//点云边缘文件地址
	string edgePath_;
	//需要保存的文件地址
	string savePath_;
	//文件名字带后缀
	string fileName_;
	//文件名字不带后缀
	string name_;
private:
	//核心函数，执行点云简化
	void PointCloudSimplification();
	//点云体素估计半径
	void VoxelRadius();
	//点数粗控制
	void NumberControl();
	//vcg向pcl格式转换
	void VcgToPcl();
	//点数精准控制
	void NumberDelete();
	//点云法向估计
	void EstimateNormal();
	//计算切空间需要的邻居索引
	void CalculateKNN();
	pcl::PointXYZ CalPlaneLineIntersectPoint(vcg::Point3f planeVector, pcl::PointXYZ planePoint, vcg::Point3f lineVector, pcl::PointXYZ linePoint);
	//切空间优化
	void TangentSmoothing();
	//旋转平面
	void RotationPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float theta, vcg::Point3f rotationAxis);
	//mls拉回
	vector<float> mls_back(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	//输出结果点云
	void OutputCloud();
	//清空所有向量
	void ClearAll();
private:
	//VCG对象
	Load_Resampling lr_;
	//当前点云xyz坐标
	vector<vcg::Point3f> xyz_;
	//当前泊松盘半径
	float poissonRadius_;
	//当前点云个数
	float pointNum_;
	//PCl格式的点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
	//点云法向
	vector<vcg::Point3f> normal_;
	//每个点周围邻居点索引
	vector<vector<int>>neighborIndex_;
private:
	struct NearDist
	{
	public:
		NearDist() = default;
		NearDist(int firstIndex, int secondIndex, float dist) :
			firstIndex_{ firstIndex }, secondIndex_{ secondIndex }, dist_{ dist }
		{};
		int firstIndex_;
		int secondIndex_;
		float dist_;
	};
};

