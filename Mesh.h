#pragma once
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>


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
