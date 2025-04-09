#pragma once
#include "DeltaMush.h"
#include "../Collison.h"
#include <set>


class ImprovedDeltaMush : public DeltaMush {
	Q_OBJECT
public:
	ImprovedDeltaMush() = default;
	void execute(std::shared_ptr<BaseMesh> basemesh, std::vector<Bone>& bones) override;
	void animatemesh(std::shared_ptr<BaseMesh> basemesh, std::vector<Bone>& bones, bool inv = false) override;
	~ImprovedDeltaMush() = default;
private:
	MyMesh meshm;
	Collison colisonhandler; // maybe for debug mode
	std::set<MyMesh::VertexHandle> colliedverteces;
	std::set<MyMesh::FaceHandle> colliedfaces;
	std::set<MyMesh::EdgeHandle> colliededges;
	void SetDistance(std::shared_ptr<BaseMesh> basemesh, std::vector<Bone>& bones);
	void executeCCD(std::shared_ptr<BaseMesh> basemesh);
	void smoothcollison(std::set<MyMesh::VertexHandle> verteces, MyMesh& mesh);
	void projectPointToPlane(const MyMesh::Point& P, const MyMesh::Normal& N, MyMesh::Point& Q);
	void DeltaMush2(std::vector<Eigen::Vector4d> v, std::shared_ptr<BaseMesh> basemesh);

};