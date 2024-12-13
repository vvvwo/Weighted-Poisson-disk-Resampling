#pragma once
#include "PCSimplification.h"
#include <pcl/io/pcd_io.h>  //文件输入输出
#include <pcl/point_types.h>  //点类型相关定义
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/transforms.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/extract_indices.h>
#include "voronoi_calculate.hpp"
#include <set>

PCSimplification::PCSimplification(string fileBase, string saveBase) :
	fileBase_{ fileBase },
	saveBase_{ saveBase }
{
}

void PCSimplification::SetResultNumber(int simplification)
{
	simplification_ = simplification;
}

void PCSimplification::SetVoronoiNumber(int voronoiNum)
{
	voronoiNum_ = voronoiNum;
}

void PCSimplification::SetVoronoi(bool voronoiFlag)
{
	voronoiFlag_ = voronoiFlag;
}

void PCSimplification::SetEdge(string edgeBase)
{
	edgeFlag_ = true;
	edgeBase_ = edgeBase;
}

void PCSimplification::SetVoronoiKNN(int K)
{
	K_ = K;
}

void ReadCloudXYZFromTxt(const std::string& file_path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	std::ifstream file(file_path.c_str());//c_str()：生成一个const char*指针，指向以空字符终止的数组。
	std::string line;
	pcl::PointXYZ point;
	while (getline(file, line)) {
		std::stringstream ss(line);
		ss >> point.x;
		ss >> point.y;
		ss >> point.z;
		cloud->push_back(point);
	}
	file.close();
}

void PCSimplification::VoxelRadius()
{
	VcgToPcl();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZ>);
	float voxelSize = 0.05;
	pcl::VoxelGrid<pcl::PointXYZ> fil;
	fil.setInputCloud(cloud_);
	fil.setLeafSize(voxelSize, voxelSize, voxelSize);
	fil.filter(*cloud_filter);
	int filterPointSize = cloud_filter->points.size();
	float V = voxelSize * voxelSize * filterPointSize * 1.47;
	poissonRadius_ = pow(V / (simplification_ * 3.1415), 1.0 / 2);
}

void PCSimplification::NumberControl() {
	float rate = pointNum_ / simplification_;
	int flag, overFlag = 0;
	float r_rate;
	//int controlIter = 0;
	//time_t iterStart2, iterEnd2;
	while (true)
	{
		//iterStart2 = clock();
		flag = 0;
		if (rate < 1)
		{
			r_rate = (1 - rate) / 1.8;
			poissonRadius_ = poissonRadius_ * (1 - r_rate);
			xyz_ = lr_.Load_Resampling_Simplify(simplification_, poissonRadius_);
			pointNum_ = lr_.nowNum;
			rate = pointNum_ / simplification_;
		}
		else
		{
			flag++;
		}
		if (rate > 1.05)
		{
			overFlag++;
			r_rate = (rate - 1) / (3 + overFlag * 0.5);
			poissonRadius_ = poissonRadius_ * (1 + r_rate);
			xyz_ = lr_.Load_Resampling_Simplify(simplification_, poissonRadius_);
			pointNum_ = lr_.nowNum;
			rate = pointNum_ / simplification_;
		}
		else
		{
			flag++;
		}
		//iterEnd2 = clock();
		//std::cout << "time: " << iterEnd2 - iterStart2 << " ms ";
		if (flag == 2)
		{
			break;
		}
		//controlIter++;
	}
	//cout << controlIter << endl;
}

void PCSimplification::VcgToPcl()
{
	cloud_->clear();
	pcl::PointXYZ point;
	for (int i = 0; i < xyz_.size(); i++)
	{
		point.x = xyz_[i][0];
		point.y = xyz_[i][1];
		point.z = xyz_[i][2];
		cloud_->push_back(point);
	}
}

void PCSimplification::NumberDelete()
{
	VcgToPcl();
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;  //建立kdtree对象
	kdtree.setInputCloud(cloud_); //设置需要建立kdtree的点云指针
	//K近邻搜索
	int K = 2;  //设置需要查找的近邻点个数
	pcl::PointXYZ searchPoint; //设置查找点
	vector<int> pointIdxNKNSearch(K);  //保存每个近邻点的索引
	vector<float> pointNKNSquaredDistance(K); //保存每个近邻点与查找点之间的欧式距离平方

	std::vector<NearDist> ND;
	ND.reserve(cloud_->points.size());
	for (int i = 0; i < cloud_->points.size(); i++)
	{
		searchPoint = cloud_->points[i];
		kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
		ND.push_back(NearDist(i, pointIdxNKNSearch[1], pointNKNSquaredDistance[1]));
	}
	std::sort(ND.begin(), ND.end(), [](const NearDist& ndA, const NearDist& ndB) {
		return ndA.dist_ < ndB.dist_;
		});
	//删除点索引
	std::set<int> deleteIndex;
	for (int i = 0; i < xyz_.size(); i++)
	{
		//如果删除点数满足要求，就可以退出
		if (deleteIndex.size() == xyz_.size() - simplification_)
		{
			break;
		}
		//看一下最小距离的两点是否出现过，如果没有出现过就删除
		if (deleteIndex.find(ND[i].firstIndex_) != deleteIndex.end() ||
			deleteIndex.find(ND[i].secondIndex_) != deleteIndex.end())
		{
			continue;
		}
		else
		{
			deleteIndex.insert(ND[i].firstIndex_);
		}
	}
	for (auto idx= deleteIndex.rbegin();idx!=deleteIndex.rend();idx++)
	{
		xyz_[*idx] = xyz_.back();
		xyz_.pop_back();
	}
	VcgToPcl();
}

void PCSimplification::EstimateNormal()
{
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
	ne.setNumberOfThreads(12);
	ne.setInputCloud(cloud_);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setKSearch(K_);
	ne.compute(*cloud_normals);
	//法向归一化
	normal_.reserve(cloud_normals->size());
	for (int i = 0; i < cloud_normals->size(); i++)
	{
		vcg::Point3f normal_i;
		double dis_i = sqrt((cloud_normals->at(i).normal_x) * (cloud_normals->at(i).normal_x) +
			(cloud_normals->at(i).normal_y) * (cloud_normals->at(i).normal_y) +
			(cloud_normals->at(i).normal_z) * (cloud_normals->at(i).normal_z));
		normal_i[0] = cloud_normals->at(i).normal_x / dis_i;
		normal_i[1] = cloud_normals->at(i).normal_y / dis_i;
		normal_i[2] = cloud_normals->at(i).normal_z / dis_i;
		normal_.push_back(normal_i);
	}
}

void PCSimplification::CalculateKNN()
{
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;  //建立kdtree对象
	kdtree.setInputCloud(cloud_); //设置需要建立kdtree的点云指针
	//K近邻搜索
	pcl::PointXYZ searchPoint; //设置查找点
	vector<int> pointIdxNKNSearch(K_);  //保存每个近邻点的索引
	vector<float> pointNKNSquaredDistance(K_); //保存每个近邻点与查找点之间的欧式距离平方
	neighborIndex_.reserve(cloud_->points.size());
	for (int i = 0; i < cloud_->points.size(); i++)
	{
		searchPoint = cloud_->points[i];
		kdtree.nearestKSearch(searchPoint, K_, pointIdxNKNSearch, pointNKNSquaredDistance);
		neighborIndex_.push_back(pointIdxNKNSearch);
	}
}

pcl::PointXYZ PCSimplification::CalPlaneLineIntersectPoint(vcg::Point3f planeVector, pcl::PointXYZ planePoint, vcg::Point3f lineVector, pcl::PointXYZ linePoint)
{
	pcl::PointXYZ returnResult;
	float vp1, vp2, vp3, n1, n2, n3, v1, v2, v3, m1, m2, m3, t, vpt;
	vp1 = planeVector[0]; vp2 = planeVector[1]; vp3 = planeVector[2];
	n1 = planePoint.x; n2 = planePoint.y; n3 = planePoint.z;
	v1 = lineVector[0]; v2 = lineVector[1]; v3 = lineVector[2];
	m1 = linePoint.x; m2 = linePoint.y; m3 = linePoint.z;
	vpt = v1 * vp1 + v2 * vp2 + v3 * vp3;
	t = ((n1 - m1) * vp1 + (n2 - m2) * vp2 + (n3 - m3) * vp3) / vpt;
	returnResult.x = m1 + v1 * t; returnResult.y = m2 + v2 * t; returnResult.z = m3 + v3 * t;
	return returnResult;
}

void PCSimplification::RotationPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float theta, vcg::Point3f rotationAxis)
{
	Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
	Eigen::AngleAxisd rotation_vector(theta, Eigen::Vector3d(rotationAxis[0], rotationAxis[1], rotationAxis[2])); //沿 指定 轴旋转 theta 度
	T.rotate(rotation_vector);
	// 执行旋转变换，并将结果保存在新创建的 cloud 中
	pcl::transformPointCloud(*cloud, *cloud, T.matrix());
}

void PCSimplification::TangentSmoothing()
{
	for (int voronoiCount = 0; voronoiCount < voronoiNum_; voronoiCount++)
	{
		#pragma omp parallel for num_threads(8)
		for (int i = 0; i < cloud_->points.size(); i++)
		{
			
			pcl::PointCloud<pcl::PointXYZ>::Ptr tan_points(new pcl::PointCloud<pcl::PointXYZ>());
			pcl::PointCloud<pcl::PointXYZ>::Ptr mlsPoints(new pcl::PointCloud<pcl::PointXYZ>());
			for (int j = 0; j < K_; j++)
			{
				pcl::PointXYZ && point = CalPlaneLineIntersectPoint(normal_[i], cloud_->points[i],
					normal_[i], cloud_->points[neighborIndex_[i][j]]);
				tan_points->push_back(point);
				//mlsPoints->push_back(cloud_->points[neighborIndex_[i][j]]);
			}

			//将切平面旋转到xoy平面
			vcg::Point3f yxis(0, 0, 1);
			float v1v2 = yxis.dot(normal_[i]);
			float theta = std::acos(v1v2);
			vcg::Point3f rotation_axis = (normal_[i]^yxis).normalized();
			if (v1v2 != 1 && v1v2 != -1)
			{
				RotationPlane(tan_points, theta, rotation_axis);
			}
			//voronoi图计算，并将点移动到胞腔中心
			voronoi_calculate_nopaint(tan_points);
			//将xoz平面旋转到切平面
			if (v1v2 != 1 && v1v2 != -1)
			{
				RotationPlane(tan_points, -theta, rotation_axis);
			}
			//mlsPoints->points[0].x = tan_points->points[0].x;
			//mlsPoints->points[0].y = tan_points->points[0].y;
			//mlsPoints->points[0].z = tan_points->points[0].z;
			//将Voronoi优化过的点云进行mls拉回
			//vector<float> mlsPoint = mls_back(mlsPoints);
			//xyz_[i][0] = mlsPoint[0];
			//xyz_[i][1] = mlsPoint[1];
			//xyz_[i][2] = mlsPoint[2];
			xyz_[i][0] = tan_points->points[0].x;
			xyz_[i][1] = tan_points->points[0].y;
			xyz_[i][2] = tan_points->points[0].z;
		}
		for (size_t p = 0; p < cloud_->points.size(); p++)
		{
			cloud_->points[p].x = xyz_[p][0];
			cloud_->points[p].y = xyz_[p][1];
			cloud_->points[p].z = xyz_[p][2];
		}
	}
}

vector<float> PCSimplification::mls_back(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	vector<float>mls_point;
	// 创建KD树
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	// 输出文件中有PointNormal类型，用来存储移动最小二乘法算出的法线
	pcl::PointCloud<pcl::PointNormal>::Ptr mls_points_normal(new pcl::PointCloud<pcl::PointNormal>);
	// 定义对象 (第二种定义类型是为了存储法线, 即使用不到也需要定义出来)
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
	mls.setInputCloud(cloud);
	mls.setComputeNormals(true);     // 是否计算法线，设置为true则计算法线
	//mls.setPolynomialFit(true);      // 设置为true则在平滑过程中采用多项式拟合来提高精度
	mls.setPolynomialOrder(2);       // 设置MLS拟合的阶数，默认是2
	mls.setSearchMethod(tree); 
	mls.setSearchRadius(1e+10);      // 邻域搜索半径
	mls.setNumberOfThreads(8);       // 设置多线程加速的线程数
	mls.setProjectionMethod(pcl::MLSResult::ProjectionMethod::ORTHOGONAL);
	mls.process(*mls_points_normal); // 曲面重建

	//pcl::PointCloud<pcl::PointXYZ>::Ptr resultCloud(new pcl::PointCloud<pcl::PointXYZ>);
	//for (int i=0;i<mls_points_normal->points.size();i++)
	//{
	//	pcl::PointXYZ tempPoint;
	//	tempPoint.x = mls_points_normal->points[i].x;
	//	tempPoint.y = mls_points_normal->points[i].y;
	//	tempPoint.z = mls_points_normal->points[i].z;
	//	resultCloud->push_back(tempPoint);
	//}
	mls_point.push_back(mls_points_normal->points[0].x);
	mls_point.push_back(mls_points_normal->points[0].y);
	mls_point.push_back(mls_points_normal->points[0].z);
	return mls_point;
}

void PCSimplification::OutputCloud()
{
	ofstream f1(savePath_);

	f1 << "ply" << endl;
	f1 << "format ascii 1.0" << endl;
	f1 << "comment VCGLIB generated" << endl;
	f1 << "element vertex " << cloud_->points.size() << endl;
	f1 << "property float x" << endl;
	f1 << "property float y" << endl;
	f1 << "property float z" << endl;	
	f1 << "element face 0" << endl;
	f1 << "property list uchar int vertex_indices" << endl;
	f1 << "end_header" << endl;

	for (int i = 0; i < cloud_->points.size(); i++) {
		f1 << cloud_->points[i].x << " " << cloud_->points[i].y << " " << cloud_->points[i].z << endl;
	}	

	f1.close();
}

void PCSimplification::ClearAll()
{
	normal_.clear();
	neighborIndex_.clear();
	xyz_.clear();
}

void PCSimplification::PointCloudSimplification()
{
	//统计时间
	time_t start{ -1 }, end{ -1 };
	start = clock();
	//点云初始化
	cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	//读取文件
	lr_.Load_Resampling_init(filePath_);
	xyz_ = lr_.pointOriginal;

	cout << "Poisson radius estimation." << endl;

	//估算泊松盘半径
	VoxelRadius();
	//为了使第一次采样尽可能大于目标值，从而使得简化点数，减小第一次的r
	poissonRadius_ = poissonRadius_ / 1.2;
	time_t iterStart1, iterEnd1;
	//iterStart1 = clock();
	xyz_ = lr_.Load_Resampling_Simplify(simplification_, poissonRadius_);
	//iterEnd1 = clock();
	//std::cout << "time: " << iterEnd1 - iterStart1 << " ms ";
	pointNum_ = lr_.nowNum;

	cout << "Delete points." << endl;

	NumberControl();
	//std::cout << pointNum_ << endl;

	NumberDelete();

	cout << "Tangenet Smoothing." << endl;

	if (voronoiFlag_ == true)
	{
		EstimateNormal();
		CalculateKNN();
		TangentSmoothing();
	}
	for (int i=0;i<cloud_->points.size();i++)
	{
		cloud_->points[i].x = ((cloud_->points[i].x + 1) / 2 * (lr_.maxV - lr_.minV)) + lr_.minV;
		cloud_->points[i].y = ((cloud_->points[i].y + 1) / 2 * (lr_.maxV - lr_.minV)) + lr_.minV;
		cloud_->points[i].z = ((cloud_->points[i].z + 1) / 2 * (lr_.maxV - lr_.minV)) + lr_.minV;
	}
	if (edgeFlag_==true)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr edgeCloud;
		edgeCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
		ReadCloudXYZFromTxt(edgePath_, edgeCloud);

		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;  //建立kdtree对象
		kdtree.setInputCloud(cloud_); //设置需要建立kdtree的点云指针
		//K近邻搜索
		pcl::PointXYZ searchPoint; //设置查找点
		vector<int> pointIdxNKNSearch;  //保存每个近邻点的索引
		vector<float> pointNKNSquaredDistance; //保存每个近邻点与查找点之间的欧式距离平方

		pcl::PointIndices indices;
		std::set<int>delestIndices;
		for (int i = 0; i < edgeCloud->points.size(); i++)
		{
			searchPoint = edgeCloud->points[i];
			kdtree.radiusSearch(searchPoint, poissonRadius_/2, pointIdxNKNSearch, pointNKNSquaredDistance);
			for (auto &idx: pointIdxNKNSearch)
			{
				delestIndices.insert(idx);
			}
		}
		for (auto &idx: delestIndices)
		{
			indices.indices.push_back(idx);
		}
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud;
		outputCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
		extract.setInputCloud(cloud_);
		extract.setNegative(true);
		extract.setIndices(std::make_shared<pcl::PointIndices>(indices));
		extract.filter(*cloud_);

		for (auto &pointXYZ:edgeCloud->points)
		{
			cloud_->push_back(pointXYZ);
		}
	}
	end = clock();

	cout << "Output." << endl;

	OutputCloud();
	ClearAll();	
	std::cout << "time: " << end - start << " ms" << std::endl;
}

int PCSimplification::doSimplification()
{
	_finddata64i32_t fileInfo;
	stringstream ss;
	ss << fileBase_;
	ss << "*";
	intptr_t hFile = _findfirst(ss.str().c_str(), &fileInfo);
	//前两次遍历非文件路径，需要跳过前两次
	int skip_2 = 0;
	if (hFile == -1)
	{
		return -1;
	}
	do
	{
		if (skip_2 < 2)
		{
			skip_2 = skip_2 + 1;
			continue;
		}
		fileName_ = fileInfo.name;
		name_ = fileName_.substr(0, fileName_.rfind("."));
		filePath_ = fileBase_ + fileName_;
		if (edgeFlag_==true)
		{
			edgePath_ = edgeBase_ + name_ + ".txt";
		}
		savePath_ = saveBase_ + name_ + ".ply";
		//代码开始
		//cout << name_ << endl;
		PointCloudSimplification();
		//代码结束
	} while (_findnext(hFile, &fileInfo) == 0);
	return 0;
}
