#include "MyViewer.h"



void MyViewer::DirectMush()
{
    int n = mesh.n_vertices();
    Eigen::SparseMatrix<double> U(4, n);
    
    for (auto v : mesh.vertices())
    {
        U.coeffRef(0, v.idx()) = mesh.point(v)[0];
        U.coeffRef(1, v.idx()) = mesh.point(v)[1];
        U.coeffRef(2, v.idx()) = mesh.point(v)[2];
        U.coeffRef(3, v.idx()) = 1;
    }
    Eigen::SparseMatrix<double> L;
    createL(L);
    Eigen::SparseMatrix<double> D(n, n);
    D.makeCompressed();
    for (auto v : mesh.vertices())
    {
        D.coeffRef(v.idx(), v.idx()) = L.coeffRef(v.idx(), v.idx());
    }
    D.makeCompressed();
    Eigen::SimplicialLDLT< Eigen::SparseMatrix<double>> solver;
    Eigen::SparseMatrix<double> I(n, n);
    I.setIdentity();
    solver.compute(D);
    double smootingfactor = 0.5;
    auto D_inv = solver.solve(I);
    auto _L = L * D_inv;
    A = (I - smootingfactor * _L);
    auto _U = U * A;
    for (int i = 0; i < n; i++)
    {

        Eigen::VectorXd u = U.col(i);
        Eigen::VectorXd _u =_U.col(i);
        Eigen::VectorXd d = u -_u;
        delt.push_back(Vec4(d[0],d[1],d[2],0));
    }

}


void MyViewer::AnDirectMush()
{


    int n = mesh.n_vertices();
    Eigen::SparseMatrix<double> V(4, n);

    for (auto v : mesh.vertices())
    {
        V.coeffRef(0, v.idx()) = mesh.point(v)[0];
        V.coeffRef(1, v.idx()) = mesh.point(v)[1];
        V.coeffRef(2, v.idx()) = mesh.point(v)[2];
        V.coeffRef(3, v.idx()) = 1;
    }
    auto _V = V * A;

    for (auto v : mesh.vertices())
    {
        Mat4 R = mesh.data(v).M;
        R.rows[3] = Vec4(0, 0, 0, 0);
        delt[v.idx()] = delt[v.idx()] * R;
        Eigen::VectorXd _v = _V.col(v.idx());
        Eigen::VectorXd d;
        d.resize(4);
        d[0] = delt[v.idx()].x;
        d[1] = delt[v.idx()].y;
        d[2] = delt[v.idx()].z;
        d[3] = 0;
        auto x = _v + d;

        mesh.point(v) = MyMesh::Point(x[0], x[1], x[2]);

    }

}


void MyViewer::createL_smooot(MyMesh& m)
{
    double smootingfactor = 0.5;
    auto mesh_ = m;
    for (int i = 0; i < 2; i++)
    {
        auto smooth = mesh_;
        for (auto v : m.vertices()) {

            Vec Avg;
            int n = 0;
            for (auto vi : m.vv_range(v)) {
                Vec vertex = Vec(mesh_.point(vi));
                Avg += vertex;
                n++;
            }
            Avg /= n;
            MyMesh::Point pointavg = MyMesh::Point(Avg.x, Avg.y, Avg.z);

            smooth.point(v) += smootingfactor * (pointavg - mesh_.point(v));

        }

        for (auto v : m.vertices()) {
            mesh_.point(v) = smooth.point(v);
        }
    }
    m = mesh_;
}


bool MyViewer::is_border_vertex(MyMesh::VertexHandle& vh) {
    return mesh.is_boundary(vh);
}



MyMesh MyViewer::smoothvectors(std::vector<Vec>& smoothed)
{
    double smootingfactor = 0.5;
    auto size = mesh.n_vertices();
    smoothed.resize(size);
    auto mesh_ = MushHelper;
    for (int i = 0; i < 20; i++)
    {
        auto smooth = mesh_;
        for (auto v : mesh.vertices()) {
            if (!is_border_vertex(v))
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
    return mesh_;
}

void MyViewer::smoothoriginal(std::vector<Vec>& smoothed)
{
    double smootingfactor = 0.5;
    for (int i = 0; i < 10; i++)
    {
        auto smooth = mesh;
        for (auto v : mesh.vertices()) {
            Vec Avg;
            int n = 0;
            for (auto vi : mesh.vv_range(v)) {
                Vec vertex = Vec(mesh.data(vi).original);
                Avg += vertex;
                n++;
            }
            Avg /= n;
            MyMesh::Point pointavg = MyMesh::Point(Avg.x, Avg.y, Avg.z);

            smooth.point(v) += smootingfactor * (pointavg - mesh.data(v).original);

        }

        for (auto v : smooth.vertices()) {
            smoothed.push_back(Vec(smooth.point(v)));
        }
    }



}





void MyViewer::Delta_Mush(std::vector<Eigen::Vector4d>& v)
{
    
    std::vector<Vec> smoothed;
    auto smooth_mesh = smoothvectors(smoothed);
    int size = smoothed.size();
    smooth_mesh.update_normals();
    for (auto ve : mesh.vertices()) {

        Eigen::MatrixXd R;
        Eigen::FullPivLU< Eigen::MatrixXd> solver;
        Eigen::Vector4d p_vector;
        Eigen::Vector4d v_vector;
        p_vector << mesh.point(ve)[0], mesh.point(ve)[1], mesh.point(ve)[2], 1;
        R.resize(4, 4);
        MyMesh::Normal normal = smooth_mesh.normal(ve);
        Vec t;
        Vec b;
        for (MyMesh::VertexOHalfedgeIter voh_it = smooth_mesh.voh_iter(ve); voh_it.is_valid(); ++voh_it) {
            MyMesh::HalfedgeHandle heh = *voh_it;
            auto ed = smooth_mesh.calc_edge_vector(heh);
            t = Vec((ed - (ed | normal) * normal).normalize());

            b = (t ^ Vec(normal)).unit();

        }
        R(0, 0) = t[0];
        R(0, 1) = t[1];
        R(0, 2) = t[2];
        R(0, 3) = 0;

        R(1, 0) = normal[0];
        R(1, 1) = normal[1];
        R(1, 2) = normal[2];
        R(1, 3) = 0;

        R(2, 0) = b[0];
        R(2, 1) = b[1];
        R(2, 2) = b[2];
        R(2, 3) = 0;

        R(3, 0) = smoothed[ve.idx()].x;
        R(3, 1) = smoothed[ve.idx()].y;
        R(3, 2) = smoothed[ve.idx()].z;
        R(3, 3) = 1;

        Eigen::MatrixXd I(4, 4);
        I.setIdentity();
        solver.compute(R.transpose());

        auto R_inv = solver.solve(I);

        v_vector = R_inv * p_vector;

        v.push_back(v_vector);


    }
    
}



void MyViewer::Delta_Mush_two(std::vector<Eigen::Vector4d> v) 
{
 for (int i = 0; i < v.size(); i++)
    {
        v[i][0] *= deltaMushFactor;
        v[i][1] *= deltaMushFactor;
        v[i][2] *= deltaMushFactor;
    }
    auto m = MushHelper;
    std::vector<Vec> smoothed;
    auto smooth_mesh = smoothvectors(smoothed);
    smooth_mesh.update_normals();
    int size = smoothed.size();
    for (auto ve : m.vertices()) {
        Eigen::MatrixXd C(4, 4);
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
        C(0, 0) = t[0];
        C(0, 1) = t[1];
        C(0, 2) = t[2];
        C(0, 3) = 0;

        C(1, 0) = normal[0];
        C(1, 1) = normal[1];
        C(1, 2) = normal[2];
        C(1, 3) = 0;

        C(2, 0) = b[0];
        C(2, 1) = b[1];
        C(2, 2) = b[2];
        C(2, 3) = 0;

        C(3, 0) = smoothed[ve.idx()].x;
        C(3, 1) = smoothed[ve.idx()].y;
        C(3, 2) = smoothed[ve.idx()].z;
        C(3, 3) = 1;

        auto d = C.transpose() * v[ve.idx()];

        mesh.point(ve) = MyMesh::Point(d[0], d[1], d[2]);


    }

   // createL_smooot(mesh);
   
}