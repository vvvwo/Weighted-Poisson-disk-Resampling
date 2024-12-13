#pragma once
#include "PcLoad.hpp"
#include <pcl/kdtree/kdtree_flann.h>  //kdtree��������
using namespace std;
class PCSimplification
{
public:
	//���캯��  ��Ҫ�����ļ���·���������ļ���·��
	PCSimplification(string fileBase, string saveBase);
	//���Ƽ�ִ��
	int doSimplification();
public:
	//���ü򻯵���Ŀ�����
	void SetResultNumber(int simplification);
	//�����пռ�ƽ��������Ĭ��Ϊ5��
	void SetVoronoiNumber(int voronoiNum);
	//�пռ��Ż��ھӸ��� (K-1)��Ĭ��K=13
	void SetVoronoiKNN(int K);
	//�����Ƿ�ִ���пռ�ƽ����Ĭ��ִ��
	void SetVoronoi(bool voronoiFlag);
	//�����Ƿ������������
	void SetEdge(string edgeBase);
private:
	//���Ƽ򻯵���
	int simplification_{ 10000 };
	//�пռ�ƽ��������Ĭ��5��
	int voronoiNum_{5};
	//�пռ��Ż��ھӸ��� (K-1)��Ĭ��K=13
	int K_{ 13 };
	//�Ƿ�ִ���пռ�ƽ��,Ĭ��ִ��
	bool voronoiFlag_{ true };
	//�Ƿ�ִ���������֣�Ĭ�ϲ�ִ��
	bool edgeFlag_{ false };
private:
	//����ģ���ļ��е�ַ
	string fileBase_;
	//���Ʊ�Ե�ļ��е�ַ
	string edgeBase_;
	//��Ҫ������ļ��е�ַ
	string saveBase_;
	//���������Ҫ������ļ��е�ַ
	string errorSaveBase_;
	//����ģ���ļ���ַ
	string filePath_;
	//���Ʊ�Ե�ļ���ַ
	string edgePath_;
	//��Ҫ������ļ���ַ
	string savePath_;
	//�ļ����ִ���׺
	string fileName_;
	//�ļ����ֲ�����׺
	string name_;
private:
	//���ĺ�����ִ�е��Ƽ�
	void PointCloudSimplification();
	//�������ع��ư뾶
	void VoxelRadius();
	//�����ֿ���
	void NumberControl();
	//vcg��pcl��ʽת��
	void VcgToPcl();
	//������׼����
	void NumberDelete();
	//���Ʒ������
	void EstimateNormal();
	//�����пռ���Ҫ���ھ�����
	void CalculateKNN();
	pcl::PointXYZ CalPlaneLineIntersectPoint(vcg::Point3f planeVector, pcl::PointXYZ planePoint, vcg::Point3f lineVector, pcl::PointXYZ linePoint);
	//�пռ��Ż�
	void TangentSmoothing();
	//��תƽ��
	void RotationPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float theta, vcg::Point3f rotationAxis);
	//mls����
	vector<float> mls_back(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	//����������
	void OutputCloud();
	//�����������
	void ClearAll();
private:
	//VCG����
	Load_Resampling lr_;
	//��ǰ����xyz����
	vector<vcg::Point3f> xyz_;
	//��ǰ�����̰뾶
	float poissonRadius_;
	//��ǰ���Ƹ���
	float pointNum_;
	//PCl��ʽ�ĵ���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
	//���Ʒ���
	vector<vcg::Point3f> normal_;
	//ÿ������Χ�ھӵ�����
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

