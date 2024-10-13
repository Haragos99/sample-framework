#pragma once
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include "Matrix4.h"
#include <Eigen/Eigen>
struct MyTraits : public OpenMesh::DefaultTraits {
    using Point = OpenMesh::Vec3d; // the default would be Vec3f
    using Normal = OpenMesh::Vec3d;
    VertexTraits{
      OpenMesh::Vec3d original;
      double mean;              // approximated mean curvature
      std::vector<double> weigh;
      std::vector<double> distance;
      Vec color;
      Mat4 M;
      int idx_of_closest_bone; 
      Eigen::MatrixXd C;
      bool close2joint;
    };
    VertexAttributes(OpenMesh::Attributes::Normal |
        OpenMesh::Attributes::Color | OpenMesh::Attributes::Status);
};
