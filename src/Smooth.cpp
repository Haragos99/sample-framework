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


bool MyViewer::is_border_vertex(MyMesh::VertexHandle& vh) {
    return mesh.is_boundary(vh);
}



MyMesh MyViewer::smoothvectors(std::vector<Vec>& smoothed)
{
    double smootingfactor = 0.5;
    auto size = mesh.n_vertices();
    smoothed.resize(size);
    auto mesh_ = MushHelper;
    for (int i = 0; i < 40; i++)
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
    smooth = mesh_;
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



std::vector<Eigen::Vector4d> MyViewer::setMushFactor(std::vector<Eigen::Vector4d> v)
{
    for (int i = 0; i < v.size(); i++)
    {
        v[i][0] *= 1.0 - deltaMushFactor;
        v[i][1] *= 1.0 - deltaMushFactor;
        v[i][2] *= 1.0 - deltaMushFactor;
    }
    return v;
}

BVH bvh;
void MyViewer::Delta_Mush(std::vector<Eigen::Vector4d>& v)
{
    
    std::vector<Vec> smoothed;
    auto smooth_mesh = smoothvectors(smoothed);
    int size = smoothed.size();

    smooth_mesh.request_face_normals();
    smooth_mesh.request_vertex_normals();
    smooth_mesh.update_normals();

    for (auto ve : mesh.vertices()) {

        Eigen::MatrixXd R;
        Eigen::FullPivLU< Eigen::MatrixXd> solver;
        Eigen::Vector4d p_vector;
        Eigen::Vector4d v_vector;
        auto p = mesh.point(ve);
        p_vector << p[0], p[1], p[2], 1;
        R.resize(4, 4);

        MyMesh::Normal normal = smooth_mesh.normal(ve);

        MyMesh::HalfedgeHandle heh = *smooth_mesh.voh_iter(ve);
        auto ed = smooth_mesh.calc_edge_vector(heh);
        Vec t = Vec((ed - (ed | normal) * normal).normalize());
        Vec b = (t ^ Vec(normal)).unit();

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


        R(3, 0) = smooth_mesh.point(ve)[0];
        R(3, 1) = smooth_mesh.point(ve)[1];
        R(3, 2) = smooth_mesh.point(ve)[2];
        R(3, 3) = 1;

        Eigen::MatrixXd I(4, 4);
        I.setIdentity();
        solver.compute(R.transpose());

        auto R_inv = solver.solve(I);

        v_vector = R_inv * p_vector;

        v.push_back(v_vector);


    }
    
}




void MyViewer::DeltaMush2(std::vector<Eigen::Vector4d> v)
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
    smooth_mesh.request_face_normals();
    smooth_mesh.request_vertex_normals();
    smooth_mesh.update_normals();
    int size = smoothed.size();
    for (auto ve : m.vertices()) {
        Eigen::MatrixXd C(4, 4);
        MyMesh::Normal normal = smooth_mesh.normal(ve);

        MyMesh::HalfedgeHandle heh = *smooth_mesh.voh_iter(ve);
        auto ed = smooth_mesh.calc_edge_vector(heh);
        Vec t = Vec((ed - (ed | normal) * normal).normalize());
        Vec b = (t ^ Vec(normal)).unit();

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

        C(3, 0) = smooth_mesh.point(ve)[0];
        C(3, 1) = smooth_mesh.point(ve)[1];
        C(3, 2) = smooth_mesh.point(ve)[2];
        C(3, 3) = 1;

        mesh.data(ve).C = C.transpose();

        auto d = C.transpose() * v[ve.idx()];

        mesh.point(ve) = MyMesh::Point(d[0], d[1], d[2]);


    }
}



void MyViewer::Delta_Mush_two(std::vector<Eigen::Vector4d> v) 
{
    
    igl::Timer timer;
    timer.start();
    DeltaMush2(v);
    Collison co;
    if (Mydelta)
    {
        co.init(v);
        co.colliedfaces = colliedfaces;
        co.colliedverteces = colliedverteces;
        co.colliededges = colliededges;
        emit startComputation(tr("CCD mesh..."));
        while (co.collisondetec(mesh, smooth))
        {
            DeltaMush2(v);
            float alfa = co.getAlfa();
            int percent = alfa * 100;
            emit midComputation(percent);
            co.setAlfa(0);
        }
        emit endComputation();
    }

    timer.stop();
    double tt = timer.getElapsedTimeInSec();

    emit displayMessage(std::to_string(tt).c_str());
    vert = co.verteces;
    smoothcollison(vert);
}

Eigen::Vector3f toEigenVec(const MyMesh::Point& v) {
    return Eigen::Vector3f(v[0], v[1], v[2]);
}


void MyViewer::SetDistance()
{
    int index = 0;
    float factor = 2.0;
    colliedverteces.clear();
    colliedfaces.clear();
    colliededges.clear();
    int fn = mesh.n_faces();
    int en = mesh.n_edges();
    int nv = mesh.n_vertices();
    for (auto v : mesh.vertices())
    {
        
        Vec meshpoint = Vec(mesh.point(v));
        for (auto b : skel.bones)
        {
            if (!b.isLastBone())
            {
                Vec bonepoint = skel.bones[index].end->point;
                float distance = (meshpoint - bonepoint).norm();
                
                if (distance <= skel.bones[index].lenght()/ factor)
                {

                    mesh.data(v).color = Vec(0, 1, 0);
                    colliedverteces.emplace(v);
                }
            }
            break;

        }

    }
    for (auto f : mesh.faces())
    {
        MyMesh::Point centroid;
        int vertexcount = 0;
        for (auto v : mesh.fv_range(f))
        {
            centroid += mesh.point(v);
            vertexcount++;
        }
        centroid /= static_cast<float>(vertexcount);
        for (auto b : skel.bones)
        {
            if (!b.isLastBone())
            {
                Vec bonepoint = skel.bones[index].end->point;
                float distance = (Vec(centroid) - bonepoint).norm();
                if (distance <= skel.bones[index].lenght() / factor)
                {
                    colliedfaces.emplace(f);
                }

            }
            break;
        }

    }
    for (auto e : mesh.edges())
    {
        MyMesh::EdgeHandle eh1 = e;

        // Get the start and end vertices of the first edge
        MyMesh::HalfedgeHandle heh1 = mesh.halfedge_handle(eh1, 0);  // First halfedge
        MyMesh::VertexHandle v0_1 = mesh.from_vertex_handle(heh1);   // Start vertex of edge 1
        MyMesh::VertexHandle v1_1 = mesh.to_vertex_handle(heh1);     // End vertex of edge 1
        Vec edge1 = Vec(mesh.point(v0_1));
        Vec edge2 = Vec(mesh.point(v1_1));
        
        for (auto b : skel.bones)
        {
            if (!b.isLastBone())
            {
                Vec bonepoint = skel.bones[index].end->point;
                float distance1 = (edge1 - bonepoint).norm();
                float distance2 = (edge2 - bonepoint).norm();

                if (distance1 <= skel.bones[index].lenght() / factor || distance2 <= skel.bones[index].lenght() / factor)
                {
                    colliededges.emplace(e);
                }

            }
            break;
        }


    }



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


void MyViewer::smoothcollison(std::set<MyMesh::VertexHandle> verteces)
{
    float smootingfactor = 0.4;
    for (int i = 0; i < 2; i++)
    {

        for (auto v : verteces)
        {
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
    for (auto v_idx : index)
    {
        MyMesh::VertexHandle vh = mesh.vertex_handle(v_idx);
        v[vh.idx()][0] *= deltaMushFactor;
        v[vh.idx()][1] *= deltaMushFactor;
        v[vh.idx()][2] *= deltaMushFactor;

        auto d = mesh.data(vh).C * v[vh.idx()];
        mesh.point(vh) = MyMesh::Point(d[0], d[1], d[2]);
    }
    
}