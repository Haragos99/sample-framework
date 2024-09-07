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
	MyMesh Delta_Mush_two(std::vector<Eigen::Vector4d> v);
	void Delta_Mush(std::vector<Eigen::Vector4d>& v);
	void smoothvectors(std::vector<Vec>& smoothed);
	~DeltaMush(){}
	float deltaMushFactor;

private:
	MyMesh mesh;
	MyMesh MushHelper;
	std::vector<Vec> smoothed;
};

