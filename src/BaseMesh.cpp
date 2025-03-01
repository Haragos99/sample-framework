#include "BaseMesh.h"
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Tools/Smoother/JacobiLaplaceSmootherT.hh>


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
    //
    Vec point = Vec(pos.data());
    grab(selected, point);
    mesh.set_point(MyMesh::VertexHandle(selected), pos);
    updateMesh();
}

void BaseMesh::grab(int selected, const Vec& position)
{
    MyMesh::VertexHandle selectedHandle(selected);
    Vec actualpoint =Vec(mesh.point(selectedHandle).data());
    Vec diff = position - actualpoint;
    for (auto v : mesh.vertices())
    {
        Vec point = Vec(mesh.point(v).data());
        double dis = distance(actualpoint, point);
        if (0.3 > dis)
        {
            double F = 1 - ((dis / 0.3));
            Vec newdiff = diff *F;
            mesh.point(v) += MyMesh::Point(newdiff.x, newdiff.y, newdiff.z);
        }
    }
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
    std::map<MyMesh::FaceHandle, double> face_area;
    std::map<MyMesh::VertexHandle, double> vertex_area;

    for (auto f : mesh.faces())
        face_area[f] = mesh.calc_sector_area(mesh.halfedge_handle(f));

    // Compute triangle strip areas
    for (auto v : mesh.vertices()) {
        vertex_area[v] = 0;
        mesh.data(v).mean = 0;
        for (auto f : mesh.vf_range(v))
            vertex_area[v] += face_area[f];
        vertex_area[v] /= 3.0;
    }

    // Compute mean values using dihedral angles
    for (auto v : mesh.vertices()) {
        for (auto h : mesh.vih_range(v)) {
            auto vec = mesh.calc_edge_vector(h);
            double angle = mesh.calc_dihedral_angle(h); // signed; returns 0 at the boundary
            mesh.data(v).mean += angle * vec.norm();
        }
        mesh.data(v).mean *= 0.25 / vertex_area[v];
    }
}


void BaseMesh::addKeyframes(int selected,float timeline)
{

}




void BaseMesh::datainfo()
{
    
}


void BaseMesh::fairMesh()
{
    OpenMesh::Smoother::JacobiLaplaceSmootherT<MyMesh> smoother(mesh);
    smoother.initialize(OpenMesh::Smoother::SmootherT<MyMesh>::Normal, // or: Tangential_and_Normal
        OpenMesh::Smoother::SmootherT<MyMesh>::C1);
    for (size_t i = 1; i <= 10; ++i) {
        smoother.smooth(10);
    }
}



void BaseMesh::reset()
{
    for (auto v : mesh.vertices())
    {
        mesh.point(v) = mesh.data(v).original;
        mesh.data(v).color = Vec(0, 0, 0);
    }
}



double BaseMesh::voronoiWeight(MyMesh::HalfedgeHandle in_he) {
    // Returns the area of the triangle bounded by in_he that is closest
    // to the vertex pointed to by in_he.
    if (mesh.is_boundary(in_he))
        return 0;
    auto next = mesh.next_halfedge_handle(in_he);
    auto prev = mesh.prev_halfedge_handle(in_he);
    double c2 = mesh.calc_edge_vector(in_he).sqrnorm();
    double b2 = mesh.calc_edge_vector(next).sqrnorm();
    double a2 = mesh.calc_edge_vector(prev).sqrnorm();
    double alpha = mesh.calc_sector_angle(in_he);

    if (a2 + b2 < c2)                // obtuse gamma
        return 0.125 * b2 * std::tan(alpha);
    if (a2 + c2 < b2)                // obtuse beta
        return 0.125 * c2 * std::tan(alpha);
    if (b2 + c2 < a2) {              // obtuse alpha
        double b = std::sqrt(b2), c = std::sqrt(c2);
        double total_area = 0.5 * b * c * std::sin(alpha);
        double beta = mesh.calc_sector_angle(prev);
        double gamma = mesh.calc_sector_angle(next);
        return total_area - 0.125 * (b2 * std::tan(gamma) + c2 * std::tan(beta));
    }

    double r2 = 0.25 * a2 / std::pow(std::sin(alpha), 2); // squared circumradius
    auto area = [r2](double x2) {
        return 0.125 * std::sqrt(x2) * std::sqrt(std::max(4.0 * r2 - x2, 0.0));
    };
    return area(b2) + area(c2);
}

double BaseMesh::distance(Vec p, Vec p1)
{
    double len = sqrt(pow(p.x - p1.x, 2) + pow(p.y - p1.y, 2) + pow(p.z - p1.z, 2));
    return len;
}

BaseMesh::~BaseMesh()
{

}