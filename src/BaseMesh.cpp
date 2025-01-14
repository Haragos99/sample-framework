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
void BaseMesh::addcolor(Vec& color)
{
    colors.push_back(color);
}


void BaseMesh::draw(Vis::Visualization& vis)
{
    glPolygonMode(GL_FRONT_AND_BACK, !vis.show_solid && vis.show_wireframe ? GL_LINE : GL_FILL);
    //glEnable(GL_CULL_FACE);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1, 1);
   //Toddo: Make good draw Transparent
    if (vis.show_solid || vis.show_wireframe) {
        for (auto f : mesh.faces()) {
            glBegin(GL_POLYGON);
            for (auto v : mesh.fv_range(f)) {
                Vec color;
                if (vis.type == Vis::VisualType::PLAIN)//refact this
                {
                    color = Vec(1, 1, 1);
                }
                else if(vis.type == Vis::VisualType::WEIGH)
                {
                    for (int i = 0; i < colors.size(); i++)
                    {
                        if (mesh.data(v).weigh[i] != 0)
                        {
                            color += (mesh.data(v).weigh[i] * colors[i]);
                        }
                    }
                    color = mesh.data(v).color == Vec(0, 0, 0) ? color : mesh.data(v).color;
                }
                
                glColor3d(color.x, color.y, color.z);
                glNormal3dv(mesh.normal(v).data());
                glVertex3dv(mesh.point(v).data());
            }
            glEnd();
        }
    }

    if (vis.show_solid && vis.show_wireframe) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glColor3d(0.0, 0.0, 0.0);
        glDisable(GL_LIGHTING);
        for (auto f : mesh.faces()) {
            glBegin(GL_POLYGON);
            for (auto v : mesh.fv_range(f))
                glVertex3dv(mesh.point(v).data());
            glEnd();
        }
        glEnable(GL_LIGHTING);
    }
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}
MyMesh& BaseMesh::getMesh() { return mesh; }

void BaseMesh::setMesh(MyMesh& _mesh) { mesh = _mesh; }

void BaseMesh::drawWithNames(Vis::Visualization& vis) const {
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
    updateMesh();
}




void BaseMesh::rotate(int selected, Vec angel)
{

}


void BaseMesh::animate(float time)
{

}


void BaseMesh::setFilename(std::string _filename)
{
    filename = _filename;
}

void BaseMesh::setCameraFocus(Vector& min, Vector& max)
{
    for (auto v : mesh.vertices()) {
        min.minimize(mesh.point(v));
        max.maximize(mesh.point(v));
    }

}

void BaseMesh::updateMesh()
{
    mesh.request_vertex_status();
    mesh.request_edge_status();
    mesh.request_face_status();
    updateVertexNormals();
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
    updateMesh();
    return true;
}


Vec BaseMesh::postSelection(const int p)
{
    return Vec(mesh.point(MyMesh::VertexHandle(p)).data());
}




void BaseMesh::updateMeanMinMax()
{

}


// Weights according to:
//   N. Max, Weights for computing vertex normals from facet normals.
//     Journal of Graphics Tools, Vol. 4(2), 1999.
void BaseMesh::updateVertexNormals()
{
    for (auto v : mesh.vertices()) {
        Vector n(0.0, 0.0, 0.0);
        for (auto h : mesh.vih_range(v)) {
            if (mesh.is_boundary(h))
                continue;
            auto in_vec = mesh.calc_edge_vector(h);
            auto out_vec = mesh.calc_edge_vector(mesh.next_halfedge_handle(h));
            double w = in_vec.sqrnorm() * out_vec.sqrnorm();
            n += (in_vec % out_vec) / (w == 0.0 ? 1.0 : w);
        }
        double len = n.length();
        if (len != 0.0)
            n /= len;
        mesh.set_normal(v, n);
    }

}
void BaseMesh::updateMeanCurvature()
{

}


void BaseMesh::addKeyframes(int selected,float timeline)
{

}




void BaseMesh::datainfo()
{

}



void BaseMesh::reset()
{
    for (auto v : mesh.vertices())
    {
        mesh.point(v) = mesh.data(v).original;
        mesh.data(v).color = Vec(0, 0, 0);
    }
}

BaseMesh::~BaseMesh()
{

}