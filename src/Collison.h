#pragma once
#include "Mesh.h"
#include "tight_inclusion/ccd.hpp"
#include <set>
using MyMesh = OpenMesh::TriMesh_ArrayKernelT<MyTraits>;

struct Delta {
	
public:
	MyMesh::Point getDeltaPoint(Eigen::MatrixXd& C) {
		Eigen::Vector4d  v = deltavector;
		v[0] *= toi; 
		v[1] *= toi;
		v[2] *= toi;
		auto d = C * v;
		return MyMesh::Point(d[0],d[1],d[2]);
	}
	float toi;
	bool isCollied;
	Delta(Eigen::Vector4d deltavector_, float toi_, bool isCollied_ ) : deltavector(deltavector_), toi(toi_), isCollied(isCollied_){}
private:
	Eigen::Vector4d deltavector;
};



class Collison {
		
public:
	Collison(){}
	void init(std::vector<Eigen::Vector4d> v);

	bool collisondetec(MyMesh& mesh, MyMesh& smooth);

	void test(MyMesh& mesh, MyMesh& smooth);
	std::set<MyMesh::VertexHandle> colliedverteces;
	std::set<MyMesh::FaceHandle> colliedfaces;
	std::set<MyMesh::EdgeHandle> colliededges;
	std::set<MyMesh::VertexHandle> verteces;



private:
	Eigen::Vector3f toEigenVec(const MyMesh::Point& v) {return Eigen::Vector3f(v[0], v[1], v[2]);
	}
	void setRestToi(float newtoi);
	void setMeshTio(MyMesh::VertexHandle& v, MyMesh& mesh);
	void setSmalest(MyMesh::VertexHandle& v, MyMesh::FaceHandle& f, MyMesh::EdgeHandle& e ,MyMesh& mesh);
	void restCollied();
	void smoothpoints(MyMesh& mesh);
	void projectPointToPlane(const MyMesh::Point& P, const MyMesh::Normal& N, MyMesh::Point& Q);
	std::vector<Delta> deltas;
	Eigen::Vector3f err = Eigen::Vector3f(-1, -1, -1);  // Error bounds
	float tmax;
	float tmaxiter;
	float tolerance;
	float mc;
	std::vector<float> tois;
	float smallestTio;
	float prevTio;
	float alfa;
};
