#include "MyViewer.h"
#include <igl/Timer.h>
#include "tight_inclusion/ccd.hpp"
//#include <fbxsdk.h>
#include "BVH.h"


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





Eigen::Vector3f toEigenVec(const MyMesh::Point& v) {
    return Eigen::Vector3f(v[0], v[1], v[2]);
}


void projectPointToPlane(const MyMesh::Point& P, const MyMesh::Normal& N, MyMesh::Point& Q) {
    // normalize the normal
    MyMesh::Normal norm = N;
    norm.normalize();

    // distance between the pointand the plane
    float d = dot(P - Q, norm);

    //  projected point
    Q = Q - (d * norm);
}





void MyViewer::smoothpoints()
{
    mesh.update_face_normals();
    mesh.update_vertex_normals();
    double smootingfactor = 0.5;
    for (int i = 0; i < 5; i++)
    {
        auto smooth = mesh;

        for (auto v : mesh.vertices()) {
            Vec Avg;
            int n = 0;
            for (auto vi : mesh.vv_range(v)) {

                auto point = mesh.point(vi);

                projectPointToPlane(mesh.point(v), mesh.normal(v), point);

                Vec vertex = Vec(point);
                Avg += vertex;
                n++;
            }
            Avg /= n;
            MyMesh::Point pointavg = MyMesh::Point(Avg.x, Avg.y, Avg.z);

            mesh.point(v) += smootingfactor * (pointavg - mesh.point(v));

        }


    }
}




void MyViewer::TestDelta(std::vector<Eigen::Vector4d> v)
{

    
}