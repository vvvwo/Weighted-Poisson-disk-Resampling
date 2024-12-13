#pragma once
#include<vcg/complex/complex.h>
#include<wrap/io_trimesh/import.h>
#include<wrap/io_trimesh/export.h>
#include<vcg/complex/algorithms/point_sampling.h>
#include<vcg/complex/algorithms/clustering.h>
#include<vcg/complex/algorithms/geodesic.h>
#include<string>
using namespace vcg;
using namespace std;

class MyEdge;
class MyFace;
class MyVertex;
struct MyUsedTypes : public UsedTypes<	Use<MyVertex>   ::AsVertexType,
    Use<MyEdge>     ::AsEdgeType,
    Use<MyFace>     ::AsFaceType> {};

class MyVertex : public Vertex<MyUsedTypes, vertex::Coord3f, vertex::Normal3f, vertex::VFAdj, vertex::Qualityf, vertex::Color4b, vertex::BitFlags, vertex::Mark> {};
class MyFace : public Face< MyUsedTypes, face::Mark, face::VertexRef, face::VFAdj, face::FFAdj, face::Normal3f, face::BitFlags > {};
class MyEdge : public Edge<MyUsedTypes> {};
class MyMesh : public tri::TriMesh< vector<MyVertex>, vector<MyFace>, vector<MyEdge>> {};

class Load_Resampling {

public:
    MyMesh m;
    vector<vcg::Point3f> pointOriginal;
    int nowNum;
    float maxV, minV;
public:

    void Load_Resampling_init(string pfileS) {
        char* pfile = new char[strlen(pfileS.c_str()) + 1];
        strcpy(pfile, pfileS.c_str());
        cout <<"Load file:"<< pfile << endl;
        if (tri::io::Importer<MyMesh>::Open(m, pfile) != 0)
        {
            printf("Error reading file  %s\n", pfile);
            exit(0);
        }
        int maxIdx, minIdx;
        maxV = -1e+10;
        minV = 1e+10;
		for (auto& vi : m.vert)
		{
			for (int i = 0; i < 3; i++)
			{
				if (vi.cP()[i] > maxV)
				{
					maxV = vi.cP()[i];
				}
				if (vi.cP()[i] < minV)
				{
					minV = vi.cP()[i];
				}
			}
		}
		for (auto& vi : m.vert)
		{
			for (int i = 0; i < 3; i++)
			{
				vi.P()[i] = ((vi.P()[i] - minV) / (maxV - minV)) * 2 - 1;
			}
		}
        Load_Resampling_Store();
    }

    vector<vcg::Point3f> Load_Resampling_Simplify(int simplification, float voxelgridR) {

        if (pointOriginal.size() <= simplification) {
            return pointOriginal;
        }

        MyMesh subM;
        tri::MeshSampler<MyMesh> mps(subM);
        tri::SurfaceSampling<MyMesh, tri::MeshSampler<MyMesh>>::PoissonDiskParam pp;
        pp.bestSampleChoiceFlag = false;
        //float radius1 = tri::SurfaceSampling<MyMesh, tri::MeshSampler<MyMesh>>::ComputePoissonDiskRadius(m, simplification);
        tri::SurfaceSampling<MyMesh, tri::MeshSampler<MyMesh>>::PoissonDiskPruning(mps, m, voxelgridR, pp);
        if (subM.vert.size() > simplification) {
            tri::Append<MyMesh, MyMesh>::MeshCopy(m, subM);
            tri::UpdateBounding<MyMesh>::Box(m);
            Load_Resampling_Store();
        }
        nowNum = subM.vert.size();
        return pointOriginal;
    }

private:    
    void Load_Resampling_Store() {    
        pointOriginal.clear();
        for (auto &vi : m.vert)
        {   
            pointOriginal.push_back(vi.cP());
        }
    }
};



