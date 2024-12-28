#pragma once
#include "Matrix4.h"
#include <Eigen/Eigen>
#include "Mesh.h"
#include "mclccd\BVHTree.hpp"
#include "Skinning.h"

using qglviewer::Vec;

class DeltaMush : public Skinning
{
public:
	DeltaMush(){}
	DeltaMush(MyMesh& _mesh) { mesh = _mesh; MushHelper = _mesh; deltaMushFactor = 1.0f; }
	void Delta_Mush_two(MyMesh& _mesh);
	void Delta_Mush();
	void execute(BaseMesh& basemesh, Skelton& skelton) override;
	MyMesh smoothvectors(std::vector<Vec>& smoothed, MyMesh& _mesh);
	void setHelper(MyMesh _mesh) { MushHelper = mesh; }
	Eigen::MatrixXd BuiledMatrix(MyMesh::Normal normal, Vec t,Vec b,Vec s);
	std::vector<Eigen::Vector4d> setMushFactor(std::vector<Eigen::Vector4d> v);
	void draw();
	~DeltaMush(){}
	float deltaMushFactor;

private:
	MyMesh mesh;
	MyMesh MushHelper;
	MyMesh Smooth;
	std::vector<Vec> smoothed;
	std::vector<Eigen::Vector4d> delta;
};

