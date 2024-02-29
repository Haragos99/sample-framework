#include "MyViewer.h"




void MyViewer::createL_smooot()
{
    double smootingfactor = 0.5;
    for (int i = 0; i < 10; i++)
    {
        auto smooth = mesh;
        for (auto v : mesh.vertices()) {
            Vec Avg;
            int n = 0;
            for (auto vi : mesh.vv_range(v)) {
                Vec vertex = Vec(mesh.point(vi));
                Avg += vertex;
                n++;
            }
            Avg /= n;
            MyMesh::Point pointavg = MyMesh::Point(Avg.x, Avg.y, Avg.z);

            smooth.point(v) += smootingfactor * (pointavg - mesh.point(v));

        }

        for (auto v : mesh.vertices()) {
            mesh.point(v) = smooth.point(v);
        }
    }
}


void MyViewer::smoothvectors(std::vector<Vec>& smoothed)
{
    double smootingfactor = 0.5;
    auto size = mesh.n_vertices();
    smoothed.resize(size);
    auto mesh_ = mesh;
    for (int i = 0; i < 10; i++)
    {
        auto smooth = mesh_;
        for (auto v : mesh.vertices()) {
            Vec Avg;
            int n = 0;
            for (auto vi : mesh.vv_range(v)) {
                Vec vertex = Vec(mesh.point(vi));
                Avg += vertex;
                n++;
            }
            Avg /= n;
            MyMesh::Point pointavg = MyMesh::Point(Avg.x, Avg.y, Avg.z);

            smooth.point(v) += smootingfactor * (pointavg - mesh.point(v));

        }

        for (auto v : smooth.vertices()) {
            mesh_.point(v) = smooth.point(v);
            smoothed[v.idx()] = (Vec(smooth.point(v)));
        }
    }
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
    smoothvectors(smoothed);
    int size = smoothed.size();

    for (auto ve : mesh.vertices()) {

        Eigen::SparseMatrix<double> R;
        Eigen::SimplicialLDLT< Eigen::SparseMatrix<double>> solver;
        Eigen::Vector4d p_vector;
        Eigen::Vector4d v_vector;
        p_vector << mesh.point(ve)[0], mesh.point(ve)[1], mesh.point(ve)[2], 1;
        R.resize(4, 4);
        MyMesh::Normal normal = mesh.normal(ve);
        Vec t;
        Vec b;
        for (MyMesh::VertexOHalfedgeIter voh_it = mesh.voh_iter(ve); voh_it.is_valid(); ++voh_it) {
            MyMesh::HalfedgeHandle heh = *voh_it;
            auto ed = mesh.calc_edge_vector(heh);
            t = Vec((ed - (ed | normal) * normal).normalize());

            b = (t ^ Vec(normal)).unit();

        }
        R.coeffRef(0, 0) = t[0];
        R.coeffRef(0, 1) = t[1];
        R.coeffRef(0, 2) = t[2];
        R.coeffRef(0, 3) = 0;

        R.coeffRef(1, 0) = normal[0];
        R.coeffRef(1, 1) = normal[1];
        R.coeffRef(1, 2) = normal[2];
        R.coeffRef(1, 3) = 0;


        R.coeffRef(2, 0) = b[0];
        R.coeffRef(2, 1) = b[1];
        R.coeffRef(2, 2) = b[2];
        R.coeffRef(2, 3) = 0;

        R.coeffRef(3, 0) = smoothed[ve.idx()].x;
        R.coeffRef(3, 1) = smoothed[ve.idx()].y;
        R.coeffRef(3, 2) = smoothed[ve.idx()].z;
        R.coeffRef(3, 3) = 1;

        Eigen::SparseMatrix<double> I(4, 4);
        I.setIdentity();
        solver.compute(R);

        auto R_inv = solver.solve(I);

        v_vector = R_inv * p_vector;

        v.push_back(v_vector);


    }
}



void MyViewer::Delta_Mush_two(std::vector<Eigen::Vector4d>& v)
{
    std::vector<Vec> smoothed;
    smoothvectors(smoothed);
    int size = smoothed.size();
    for (auto ve : mesh.vertices()) {
        Eigen::SparseMatrix<double> C;
        Eigen::Vector4d p_vector;
        Eigen::Vector4d v_vector;
        p_vector << mesh.point(ve)[0], mesh.point(ve)[1], mesh.point(ve)[2], 1;
        C.resize(4, 4);
        MyMesh::Normal normal = mesh.normal(ve);
        Vec t;
        Vec b;
        for (MyMesh::VertexOHalfedgeIter voh_it = mesh.voh_iter(ve); voh_it.is_valid(); ++voh_it) {
            MyMesh::HalfedgeHandle heh = *voh_it;
            auto ed = mesh.calc_edge_vector(heh);
            t = Vec((ed - (ed | normal) * normal).normalize());

            b = (t ^ Vec(normal)).unit();

        }
        C.coeffRef(0, 0) = t[0];
        C.coeffRef(0, 1) = t[1];
        C.coeffRef(0, 2) = t[2];
        C.coeffRef(0, 3) = 0;

        C.coeffRef(1, 0) = normal[0];
        C.coeffRef(1, 1) = normal[1];
        C.coeffRef(1, 2) = normal[2];
        C.coeffRef(1, 3) = 0;

        C.coeffRef(2, 0) = b[0];
        C.coeffRef(2, 1) = b[1];
        C.coeffRef(2, 2) = b[2];
        C.coeffRef(2, 3) = 0;

        C.coeffRef(3, 0) = smoothed[ve.idx()].x;
        C.coeffRef(3, 1) = smoothed[ve.idx()].y;
        C.coeffRef(3, 2) = smoothed[ve.idx()].z;
        C.coeffRef(3, 3) = 1;

        auto d = C * v[ve.idx()];

        mesh.point(ve) = MyMesh::Point(d[0], d[1], d[2]);


    }

}