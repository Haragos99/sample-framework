#include "BVH.h"
#include "tight_inclusion/ccd.hpp"

void BVH::build()
{
	vertecies.clear();
	faces.clear();
	edges.clear();

	// Iterate over all vertices and save the vertex handles
	for (const auto& vh : mesh.vertices()) {
		vertecies.insert(vh);
	}

	// Iterate over all faces and save the face handles
	for (const auto& fh : mesh.faces()) {
		faces.insert(fh);
	}

	// Iterate over all edges and save the edge handles
	for (const auto& eh : mesh.edges()) {
		edges.insert(eh);
	}

	int deep = 0;
	root = buildBVH(0, faces.size(),vertecies,faces,edges,mesh,deep);
	root2 = buildBVH(0, faces.size(), vertecies, faces, edges, s, deep);
}


// Build the binary tree
std::shared_ptr<BVHNode> BVH::buildBVH(int start, int end, std::set<MyMesh::VertexHandle> v, std::set<MyMesh::FaceHandle> f, std::set<MyMesh::EdgeHandle> e, MyMesh& m, int& deep)
{
	std::shared_ptr<BVHNode> node = std::make_shared<BVHNode>();
	std::vector<MyMesh::FaceHandle> vec(f.begin(), f.end());
	array.push_back(node);
	for (auto face :vec) {
		
		node->boundingBox.expand(createAABBforFace(face,m));
	}

	// If we are at a leaf node (small number of triangles), stop recursion
	if (end - start <= 1 || deep >40) {
		node->start = start;
		node->end = end;
		node->isLeaf = true;
		node->vertecies = v;
		node->edges = e;
		node->faces = f;
		return node;
	}
	int mid = (start + end) / 2;

	std::set<MyMesh::VertexHandle> vl, vr;
	std::set<MyMesh::FaceHandle> fl, fr;
	std::set<MyMesh::EdgeHandle> el, er;

	partitionMesh(v, f, e, vl, fl, el, vr, fr, er);
	deep++;
	node->left = buildBVH(start, mid,vl,fl,el,m,deep);
	node->right = buildBVH(mid, end,vr,fr,er,m,deep);

	// Expand the node's AABB to include both children's AABBs
	node->boundingBox.expand(node->left->boundingBox);
	node->boundingBox.expand(node->right->boundingBox);

	return node;
}

float BVH::computeMedianX() {
	std::vector<float> xPositions;

	for (auto v : mesh.vertices()) {
		xPositions.push_back(mesh.point(v)[0]); // Extract x-coordinate
	}

	std::sort(xPositions.begin(), xPositions.end());
	return xPositions[xPositions.size() / 2]; // Return median
}

// Function to split the mesh into two parts based on a heuristic
void BVH::partitionMesh(std::set<MyMesh::VertexHandle>& v, std::set<MyMesh::FaceHandle>& f, std::set<MyMesh::EdgeHandle>& e,
	std::set<MyMesh::VertexHandle>& vl, std::set<MyMesh::FaceHandle>& fl, std::set<MyMesh::EdgeHandle>& el,
	std::set<MyMesh::VertexHandle>& vr, std::set<MyMesh::FaceHandle>& fr, std::set<MyMesh::EdgeHandle>& er ) {
	// Simple heuristic: split based on median position along an axis (e.g., X-axis)
	// This is just an example; more sophisticated partitioning techniques can be used
	float medianX = computeMedianX();

	for (auto eg : e) {
		// Get the vertices of the edge
		MyMesh::HalfedgeHandle heh2 = mesh.halfedge_handle(eg, 0);  // Second halfedge
		MyMesh::VertexHandle v0 = mesh.from_vertex_handle(heh2);   // Start vertex of edge 2
		MyMesh::VertexHandle v1 = mesh.to_vertex_handle(heh2);     // End vertex of edge 2

		// Check the positions of the vertices
		bool inLeft = (mesh.point(v0)[0] <= medianX);
		bool inRight = (mesh.point(v1)[0] > medianX);

		// Add the edge to the appropriate vector
		if (inLeft) {
			el.insert(eg);
		}
		if (inRight) {
			er.insert(eg);
		}
	}

	// Loop over all faces and assign them to left or right based on their vertex positions
	for (auto fa : f) {
		bool inLeft = false, inRight = false;

		// Check the positions of the vertices in the face
		for (auto fv : mesh.fv_range(fa)) {
			if (mesh.point(fv)[0] <= medianX) {
				inLeft = true;  // Vertex is on the left side
				vl.insert(fv); // Add vertex to leftVertices
			}
			else {
				inRight = true; // Vertex is on the right side
				vr.insert(fv); // Add vertex to rightVertices
			}
		}

		// Assign the face to the appropriate side based on vertices
		if (inLeft) {
			fl.insert(fa); // Add face to leftFaces
		}
		if (inRight) {
			fr.insert(fa); // Add face to rightFaces
		}
	}

}



AABB BVH::createAABBforFace(MyMesh::FaceHandle& f, MyMesh& m)
{
	AABB box;

	std::vector<MyMesh::Point> faceVertices;
	for (const auto& fv_it : mesh.fv_range(f)) {
		box.expand(mesh.point(fv_it).data());
	}
	return box;
}


void BVH::traverse(MyMesh& me, MyMesh& smooth, std::vector<Eigen::Vector4d> vi)
{
	AABB query;
	for (auto v_it = me.vertices_begin(); v_it != me.vertices_end(); ++v_it) {
		MyMesh::Point point = me.point(*v_it);
		query.expand(point.data());  // Expand AABB to include the vertex position
	}

	traverseBVH(root, root2,me, smooth, query,vi);
}


// todo rewriet 
void BVH::traverseBVH(const std::shared_ptr<BVHNode>& node, const std::shared_ptr<BVHNode>& node2, MyMesh& me, MyMesh& smooth, AABB query, std::vector<Eigen::Vector4d>& vi)
{
	Eigen::Vector3f v_t0, v_t1;
	Eigen::Vector3f f0_t0, f1_t0, f2_t0;
	Eigen::Vector3f f0_t1, f1_t1, f2_t1;
	float toi = 0;

	Eigen::Vector3f err = Eigen::Vector3f(-1, -1, -1);  // Error bounds
	float tmax = 1.0;
	float tmaxiter = 1e7;

	float tolerance = 1e-2;
	float outtolerance;

	float mc = 1e-6;
	if (!node||!node2) {
		return;
	}
	if (node2->boundingBox.overlaps(node->boundingBox))
	{
		if (node->isLeaf && node2->isLeaf) {
			for (auto& v : node->vertecies) {
				for (auto& f : node->faces) {
					//me.data(v).color = Vec(1, 0.3, 0);


					v_t0 = toEigenVec(me.point(v));
					v_t1 = toEigenVec(smooth.point(v));

					MyMesh::FaceVertexIter fv_it = me.fv_iter(f);
					MyMesh::VertexHandle v0 = *fv_it;
					MyMesh::VertexHandle v1 = *(++fv_it);
					MyMesh::VertexHandle v2 = *(++fv_it);

					bool isInTriangle = v0 == v || v1 == v || v2 == v;

					if (isInTriangle)
					{
						continue;
					}

					f0_t0 = toEigenVec(me.point(v0));
					f0_t1 = toEigenVec(smooth.point(v0));

					f1_t0 = toEigenVec(me.point(v1));
					f1_t1 = toEigenVec(smooth.point(v1));

					f2_t0 = toEigenVec(me.point(v2));
					f2_t1 = toEigenVec(smooth.point(v2));


					bool iscollied = ticcd::vertexFaceCCD(
						v_t0, f0_t0, f1_t0, f2_t0,
						v_t1, f0_t1, f1_t1, f2_t1,
						err, mc, toi, tolerance, tmax, tmaxiter, outtolerance
					);
					if (iscollied) {
						me.data(v).color = Vec(1, 0, 0);
						vi[v.idx()][0] *= toi;
						vi[v.idx()][1] *= toi;
						vi[v.idx()][2] *= toi;
						auto di = me.data(v).C * vi[v.idx()];
						me.point(v) = MyMesh::Point(di[0], di[1], di[2]);
					}




				}
			}
		}
		else
		{
			traverseBVH(node->left, node2->left, me, smooth, query,vi);
			traverseBVH(node->right, node2->right,me, smooth, query,vi);
			traverseBVH(node->right, node2->left, me, smooth, query,vi);
			traverseBVH(node->left, node2->right, me, smooth, query,vi);
		}
		if (!node->isLeaf) {

			traverseBVH(node->left, node2, me, smooth, query,vi);
			traverseBVH(node->right, node2, me, smooth, query,vi);
		}
		else
		{
			traverseBVH(node, node2->left, me, smooth, query,vi);
			traverseBVH(node, node2->right, me, smooth, query,vi);
		}


	}
	else {

	}



	/*
	// If the query AABB does not overlap this node's AABB, exit early
	if (!node->boundingBox.overlaps(queryBox)) {
	
	}

	// If this is a leaf node, perform the actual collision detection
	if (node->start != -1 && node->end != -1) {
		for (int i = node->start; i < node->end; ++i) {
			// Vertex-Face Collision Test
			for (const Face& f1 : faces1) {
				for (const Vertex& v2 : vertices2) {
					if (vertexFaceCollision(v2, f1)) {
						return true;  // Collision detected
					}
				}
			}

			for (const Face& f2 : faces2) {
				for (const Vertex& v1 : vertices1) {
					if (vertexFaceCollision(v1, f2)) {
						return true;  // Collision detected
					}
				}
			}

			// Edge-Edge Collision Test
			for (const Edge& e1 : edges1) {
				for (const Edge& e2 : edges2) {
					if (edgeEdgeCollision(e1, e2)) {
						return true;  // Collision detected
					}
				}
			}
		}
	}

	// Traverse left and right children
	bool leftCollision = node->left ? traverseBVH(node->left, queryBox, vertices1, faces1, edges1, vertices2, faces2, edges2) : false;
	bool rightCollision = node->right ? traverseBVH(node->right, queryBox, vertices1, faces1, edges1, vertices2, faces2, edges2) : false;
*/
}