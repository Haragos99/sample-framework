#pragma once
#include <src/Mesh.h>
#include <BVHNode.h>
#include <set>
using MyMesh = OpenMesh::TriMesh_ArrayKernelT<MyTraits>;

class BVH
{
public :
	
	BVH(){}
	BVH(MyMesh _mesh, MyMesh _s) { mesh = _mesh; s = _s; }
	void build();

	std::shared_ptr<BVHNode> buildBVH(int start, int end, std::set<MyMesh::VertexHandle> v, std::set<MyMesh::FaceHandle> f, std::set<MyMesh::EdgeHandle> e, MyMesh& m,int& deep);

	void traverse(MyMesh& me, MyMesh& smooth, std::vector<Eigen::Vector4d> vi);
	std::vector<std::shared_ptr<BVHNode>> array;
private :
	Eigen::Vector3f toEigenVec(const MyMesh::Point& v) {
		return Eigen::Vector3f(v[0], v[1], v[2]);
	}
	std::shared_ptr<BVHNode> root;
	std::shared_ptr<BVHNode> root2;
	MyMesh mesh; // original
	MyMesh s; // mush
	std::set<MyMesh::VertexHandle> vertecies;
	std::set<MyMesh::FaceHandle> faces;
	std::set<MyMesh::EdgeHandle> edges;
	void traverseBVH(const std::shared_ptr<BVHNode>& node, const std::shared_ptr<BVHNode>& node2,MyMesh& me, MyMesh& smooth, AABB query, std::vector<Eigen::Vector4d>& vi);
	AABB createAABBforFace(MyMesh::FaceHandle& f,MyMesh& m);
	float computeMedianX();
	void partitionMesh(std::set<MyMesh::VertexHandle>& v, std::set<MyMesh::FaceHandle>& f, std::set<MyMesh::EdgeHandle>& e,
		std::set<MyMesh::VertexHandle>& vl, std::set<MyMesh::FaceHandle>& fl, std::set<MyMesh::EdgeHandle>& el,
		std::set<MyMesh::VertexHandle>& vr, std::set<MyMesh::FaceHandle>& fr, std::set<MyMesh::EdgeHandle>& er);


};