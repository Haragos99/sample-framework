#include "DeltaMush.h"








MyMesh DeltaMush::smoothvectors(std::vector<Vec>& smoothed, MyMesh& _mesh)
{
    double smootingfactor = 0.5;
    auto size = mesh.n_vertices();
    smoothed.resize(size);
    auto mesh_ = _mesh;
    for (int i = 0; i < 20; i++)
    {
        auto smooth = mesh_;
        for (auto v : mesh.vertices()) {
            if (!_mesh.is_boundary(v))
            {
                Vec Avg;
                int n = 0;
                for (auto vi : mesh.vv_range(v)) {
                    Vec vertex = Vec(mesh_.point(vi));
                    Avg += vertex;
                    n++;
                }
                Avg /= n;
                MyMesh::Point pointavg = MyMesh::Point(Avg.x, Avg.y, Avg.z);
                smooth.point(v) += smootingfactor * (pointavg - mesh_.point(v));
            }

        }

        for (auto v : smooth.vertices()) {
            mesh_.point(v) = smooth.point(v);
            smoothed[v.idx()] = (Vec(smooth.point(v)));
            
        }
    }
    Smooth = mesh_;
    return mesh_;
}


Eigen::MatrixXd DeltaMush::BuiledMatrix(MyMesh::Normal normal, Vec t, Vec b, Vec s)
{
    Eigen::MatrixXd M;
    M.resize(4, 4);
    M(0, 0) = t[0];
    M(0, 1) = t[1];
    M(0, 2) = t[2];
    M(0, 3) = 0;

    M(1, 0) = normal[0];
    M(1, 1) = normal[1];
    M(1, 2) = normal[2];
    M(1, 3) = 0;

    M(2, 0) = b[0];
    M(2, 1) = b[1];
    M(2, 2) = b[2];
    M(2, 3) = 0;

    M(3, 0) = s.x;
    M(3, 1) = s.y;
    M(3, 2) = s.z;
    M(3, 3) = 1;

    return M.transpose();
}



void DeltaMush::Delta_Mush()
{
    delta.clear();
    std::vector<Vec> smoothed;
    auto _mesh = smoothvectors(smoothed,mesh);
    int size = smoothed.size();
    _mesh.update_normals();
    for (auto ve : _mesh.vertices()) {

        Eigen::MatrixXd R;
        Eigen::FullPivLU< Eigen::MatrixXd> solver;
        Eigen::Vector4d p_vector;
        Eigen::Vector4d v_vector;
        p_vector << mesh.point(ve)[0], mesh.point(ve)[1], mesh.point(ve)[2], 1;
        R.resize(4, 4);
        MyMesh::Normal normal = _mesh.normal(ve);
        Vec t;
        Vec b;
        for (MyMesh::VertexOHalfedgeIter voh_it = _mesh.voh_iter(ve); voh_it.is_valid(); ++voh_it) {
            MyMesh::HalfedgeHandle heh = *voh_it;
            auto ed = _mesh.calc_edge_vector(heh);
            t = Vec((ed - (ed | normal) * normal).normalize());
            b = (t ^ Vec(normal)).unit();
        }
        R = BuiledMatrix(normal, t, b, smoothed[ve.idx()]);
        Eigen::MatrixXd I(4, 4);
        I.setIdentity();
        solver.compute(R);

        auto R_inv = solver.solve(I);

        v_vector = R_inv * p_vector;

        delta.push_back(v_vector);

    }
}


void DeltaMush::draw()
{
    if (deltaMushFactor < 1)
    {
        auto v_d = setMushFactor(delta);
        for (auto v : Smooth.vertices())
        {
            glLineWidth(10.0);
            glBegin(GL_LINES);
            glColor3d(1.0, 0.0, 0.0);
            //glVertex3dv(Smooth.point(v).data());
            Eigen::Vector4d p_vector;
            p_vector << Smooth.point(v)[0], Smooth.point(v)[1], Smooth.point(v)[2], 1;
            Eigen::Vector4d d = p_vector + v_d[v.idx()];
            //glVertex3dv(d.data());
            glColor3d(1.0, 0.0, 0.0);;
            glEnd();
        }
    }

}

std::vector<Eigen::Vector4d> DeltaMush::setMushFactor(std::vector<Eigen::Vector4d> v)
{
    for (int i = 0; i < v.size(); i++)
    {
        v[i][0] *= deltaMushFactor;
        v[i][1] *= deltaMushFactor;
        v[i][2] *= deltaMushFactor;
    }
    return v;
}
void DeltaMush::Delta_Mush_two( MyMesh& _mesh) 
{ 
    auto v_d =setMushFactor(delta);
    auto m = _mesh;
    std::vector<Vec> smoothed;
    auto smooth_mesh = smoothvectors(smoothed,_mesh);
    smooth_mesh.update_normals();
    int size = smoothed.size();
    for (auto ve : m.vertices()) {
        Eigen::MatrixXd C(4,4);
        Eigen::Vector4d p_vector;
        Eigen::Vector4d v_vector;
        p_vector << m.point(ve)[0], m.point(ve)[1], m.point(ve)[2], 1;
        MyMesh::Normal normal = smooth_mesh.normal(ve);
        Vec t;
        Vec b;
        for (MyMesh::VertexOHalfedgeIter voh_it = smooth_mesh.voh_iter(ve); voh_it.is_valid(); ++voh_it) {
            MyMesh::HalfedgeHandle heh = *voh_it;
            auto ed = smooth_mesh.calc_edge_vector(heh);
            t = Vec((ed - (ed | normal) * normal).normalize());

            b = -(t ^ Vec(normal)).unit();

        }
       
        C = BuiledMatrix(normal, t, b, smoothed[ve.idx()]);
        auto d = C * v_d[ve.idx()];
        _mesh.point(ve) = MyMesh::Point(d[0], d[1], d[2]);
    }
   // mesh = _mesh;
}