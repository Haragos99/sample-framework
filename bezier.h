#pragma once
#include "MyViewer.h"

class Bezier {
public:
	struct MyTraits : public OpenMesh::DefaultTraits {
		using Point = OpenMesh::Vec3d; // the default would be Vec3f
		using Normal = OpenMesh::Vec3d;
		VertexTraits{
		  OpenMesh::Vec3d original;
		  double mean;              // approximated mean curvature
		  std::vector<double> weigh;
		  std::vector<double> distance;
		  int idx_of_closest_bone;

		};
		VertexAttributes(OpenMesh::Attributes::Normal |
			OpenMesh::Attributes::Color | OpenMesh::Attributes::Status);
	};
	using MyMesh = OpenMesh::TriMesh_ArrayKernelT<MyTraits>;
	using Vector = OpenMesh::VectorT<double, 3>;

	Bezier(std::string filename_);
	Bezier();
	 ~Bezier();
	void draw(bool show_control_points) const;
	void drawWithNames(bool show_control_points) const;
	Vector postSelection(int selected) { return control_points[selected]; };
	
	void movement(int selected, const Vector& pos);
	void updateBaseMesh();
	bool reload();
	void set_mesh(MyMesh mesh_) { mesh = mesh_; };
private:

	MyMesh mesh;
	size_t degree[2];
	std::vector<Vector> control_points;
	std::string filename;
	
};
