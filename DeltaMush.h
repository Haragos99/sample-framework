#pragma once
#include "Matrix4.h"
#include <Eigen/Eigen>
#include "Mesh.h"

using qglviewer::Vec;
using MyMesh = OpenMesh::TriMesh_ArrayKernelT<MyTraits>;
using Vector = OpenMesh::VectorT<double, 3>;

class DeltaMush
{
public:
	DeltaMush(){}
	DeltaMush(MyMesh& _mesh) { mesh = _mesh; MushHelper = _mesh; deltaMushFactor = 1.0f; }
	MyMesh Delta_Mush_two(MyMesh _mesh);
	void Delta_Mush();
	MyMesh smoothvectors(std::vector<Vec>& smoothed, MyMesh _mesh);
	void setHelper(MyMesh _mesh) { MushHelper = mesh; }
	Eigen::MatrixXd BuiledMatrix(MyMesh::Normal normal, Vec t,Vec b,Vec s);
	~DeltaMush(){}
	float deltaMushFactor;

private:
	MyMesh mesh;
	MyMesh MushHelper;
	std::vector<Vec> smoothed;
	std::vector<Eigen::Vector4d> delta;
};

