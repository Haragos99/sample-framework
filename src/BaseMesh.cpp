#include "BaseMesh.h"
#include <OpenMesh/Core/IO/MeshIO.hh>



BaseMesh::BaseMesh(std::string filename) : Object3D(filename)
{
     open();
}

BaseMesh::BaseMesh(MyMesh& _mesh)
{
    mesh = _mesh;
}

void BaseMesh::scale(float scale)
{

}



void BaseMesh::draw(Visualization& vis)
{
   //Toddo: Make good draw Transparent
}
MyMesh& BaseMesh::getMesh() { return mesh; }

void BaseMesh::setMesh(MyMesh& _mesh) { mesh = _mesh; }

void BaseMesh::drawWithNames(Visualization& vis) const {
    if (!vis.show_wireframe)
        return;
    for (auto v : mesh.vertices()) {
        glPushName(v.idx());
        glRasterPos3dv(mesh.point(v).data());
        glPopName();
    }
}

void BaseMesh::movement(int selected, const Vector& pos) {
    mesh.set_point(MyMesh::VertexHandle(selected), pos);
}




void BaseMesh::rotate(int selected, Vec angel)
{

}


void BaseMesh::animate(float time)
{

}


bool BaseMesh::open()
{
    if (!OpenMesh::IO::read_mesh(mesh, filename) || mesh.n_vertices() == 0)
        return false;

    mesh.request_face_normals(); 
    mesh.request_vertex_normals();
    mesh.update_face_normals();
    for (auto v : mesh.vertices())
    {
        mesh.data(v).original = mesh.point(v);
    }
    mesh.request_vertex_status();
    mesh.request_edge_status();
    mesh.request_face_status();
    return true;
}


Vec BaseMesh::postSelection(const int p)
{
    return Vec(mesh.point(MyMesh::VertexHandle(p)).data());
}

BaseMesh::~BaseMesh()
{

}