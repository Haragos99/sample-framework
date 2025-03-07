#pragma once
#include "BaseMesh.h"

using PointsMap = std::map<MyMesh::VertexHandle, MyMesh::Point>;
class Blendshape {
private:
	PointsMap origanls;
	std::shared_ptr<BaseMesh> blendshapemesh;
	std::vector<std::pair<int,PointsMap>> targets;
	MyMesh::Point getBlendMove(MyMesh::VertexHandle v);
public:
	Blendshape(std::shared_ptr<BaseMesh> basemesh);
	void addNewPostion(std::shared_ptr<BaseMesh> basemesh);
	void setWeight(int i, double weigt);
	void calculate();
};