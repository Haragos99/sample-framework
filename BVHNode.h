#pragma once
#include <AABB.h>
#include <memory>
#include <vector>
#include <set>
#include <src/Mesh.h>
using MyMesh = OpenMesh::TriMesh_ArrayKernelT<MyTraits>;
struct BVHNode {
    AABB boundingBox;  // The bounding volume (an Axis-Aligned Bounding Box - AABB)
    int start, end;    // The range of indices in the mesh elements (e.g., triangles) covered by this node
    std::shared_ptr<BVHNode> left;  // Pointer to the left child node (for inner nodes)
    std::shared_ptr<BVHNode> right; // Pointer to the right child node (for inner nodes)
    std::set<MyMesh::VertexHandle> vertecies;
    std::set<MyMesh::FaceHandle> faces;
    std::set<MyMesh::EdgeHandle> edges;
    bool isLeaf = false;
    BVHNode() : start(-1), end(-1), left(nullptr), right(nullptr) {}



    ~BVHNode() = default;
};
