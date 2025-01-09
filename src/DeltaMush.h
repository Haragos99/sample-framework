#pragma once
#include "Matrix4.h"
#include <Eigen/Eigen>
#include "Mesh.h"
#include "mclccd\BVHTree.hpp"
#include "Skinning.h"
#include "Lines.h"

using qglviewer::Vec;

class DeltaMush : public Skinning
{
public:
	DeltaMush(){ deltaMushFactor = 1.0f; }
	void execute(std::shared_ptr<BaseMesh> basemesh, std::vector<Bone>& bones) override;
	virtual void animatemesh(std::shared_ptr<BaseMesh> basemesh, std::vector<Bone>& bones, bool inv = false) override;
	~DeltaMush(){}
	float deltaMushFactor;

private:
	void createDeltaline(Vec& start, Vec& end);
	void Delta_Mush_two(std::shared_ptr<BaseMesh> basemesh);
	void Delta_Mush(std::shared_ptr<BaseMesh> basemesh);
	std::shared_ptr<BaseMesh> smoothMesh(std::shared_ptr<BaseMesh> basemesh);
	Eigen::MatrixXd BuiledMatrix(MyMesh::Normal normal, Vec t,Vec b,Vec s);
	void modifyDeltaLine(std::vector<Eigen::Vector4d> deltavector);
	std::vector<Eigen::Vector4d> setMushFactor(std::vector<Eigen::Vector4d> v);
	std::vector<Eigen::Vector4d> delta;
	std::shared_ptr<DeltaLines> lines;
};

