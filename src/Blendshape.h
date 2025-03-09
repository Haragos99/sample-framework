#pragma once
#include "BaseMesh.h"
#include <set>

using PointsMap = std::map<MyMesh::VertexHandle, MyMesh::Point>;
class Blendshape {
private:
	PointsMap origanls;
	std::shared_ptr<BaseMesh> blendshapemesh;
	std::vector<std::pair<double,PointsMap>> targets;
	std::set<MyMesh::VertexHandle> modifyVerteces;
	MyMesh::Point getBlendMove(MyMesh::VertexHandle v);
public:
	Blendshape(std::shared_ptr<BaseMesh> basemesh);
	void addNewPostion(std::shared_ptr<BaseMesh> basemesh);
	void setWeight(int i, double weigt);
	void calculate();
};