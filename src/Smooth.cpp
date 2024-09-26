#include "MyViewer.h"

#include <tbb/concurrent_unordered_set.h>
#include <tbb/concurrent_vector.h>
#include <igl/Timer.h>
#include "tight_inclusion/ccd.hpp"



// Custom hash function for Eigen::Vector4i
namespace std {
    template <>
    struct hash<Eigen::Vector4i> {
        std::size_t operator()(const Eigen::Vector4i& vec) const {
            std::size_t seed = 0;
            for (int i = 0; i < 4; ++i) {
                seed ^= std::hash<int>{}(vec[i]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
            return seed;
        }
    };

    // Custom hash function for tuple<Eigen::Vector4i, int, double>
    template <>
    struct hash<std::tuple<Eigen::Vector4i, int, double>> {
        std::size_t operator()(const std::tuple<Eigen::Vector4i, int, double>& t) const {
            const auto& [vec, i, d] = t;
            std::size_t seed = 0;
            seed ^= std::hash<Eigen::Vector4i>{}(vec)+0x9e3779b9 + (seed << 6) + (seed >> 2);
            seed ^= std::hash<int>{}(i)+0x9e3779b9 + (seed << 6) + (seed >> 2);
            seed ^= std::hash<double>{}(d)+0x9e3779b9 + (seed << 6) + (seed >> 2);
            return seed;
        }
    };
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



void MyViewer::saveMeshToEigen(const MyMesh& _mesh, Eigen::MatrixXd& V)
{
    int numVertices = _mesh.n_vertices();
    V.resize(numVertices, 3);
    int i = 0;
    for (const auto& vh : _mesh.vertices()) {
        mesh.data(vh).color = Vec(0, 0, 0);
        auto point = _mesh.point(vh);
        V(i, 0) = point[0];  // x-coordinate
        V(i, 1) = point[1];  // y-coordinate
        V(i, 2) = point[2];  // z-coordinate
        ++i;
    }
}

void MyViewer::saveMeshFaceToEigen(const MyMesh& _mesh, Eigen::MatrixXi& F)
{
    int numFaces = _mesh.n_faces();
    F.resize(numFaces, 3);
    int i = 0;
    for (const auto& fh : _mesh.faces()) {
        int j = 0;
        for (const auto& vh : _mesh.fv_range(fh)) {
            F(i, j) = vh.idx();  // Store vertex index
            ++j;
        }
        ++i;
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



void MyViewer::Delta_Mush_two(std::vector<Eigen::Vector4d> v) 
{
    igl::Timer timer;
    timer.start();
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
    timer.stop();
    double tt = timer.getElapsedTimeInSec();
    collisonTest2(v);
    //collisonTest(v);
}

Eigen::Vector3f toEigenVec(const MyMesh::Point& v) {
    return Eigen::Vector3f(v[0], v[1], v[2]);
}





void MyViewer::collisonTest2(std::vector<Eigen::Vector4d> vi)
{
    tios.clear();
    Eigen::Vector3f v_t0, v_t1;
    Eigen::Vector3f f0_t0, f1_t0, f2_t0;
    Eigen::Vector3f f0_t1, f1_t1, f2_t1;
    float toi = 0;

    Eigen::Vector3f err = Eigen::Vector3f(-1, -1, -1);  // Error bounds
    float tmax = 1.0;
    float tmaxiter = 1e7;

    float tolerance =1e-6;
    float outtolerance;

    float mc = 1e-6;

    for (auto v : mesh.vertices())
    {
        v_t0 = toEigenVec(mesh.point(v));
        v_t1 = toEigenVec(smooth.point(v));


        for (auto f : mesh.faces())
        {
            MyMesh::FaceVertexIter fv_it =mesh.fv_iter(f);
            MyMesh::VertexHandle v0 = *fv_it;
            MyMesh::VertexHandle v1 = *(++fv_it);
            MyMesh::VertexHandle v2 = *(++fv_it);

            if (v0 == v || v1 == v || v2 == v)
            {
                break;
            }



            f0_t0 = toEigenVec(mesh.point(v0));
            f0_t1 = toEigenVec(smooth.point(v0));

            f1_t0 = toEigenVec(mesh.point(v1));
            f1_t1 = toEigenVec(smooth.point(v1));

            f2_t0 = toEigenVec(mesh.point(v2));
            f2_t1 = toEigenVec(smooth.point(v2));


           bool a = ticcd::vertexFaceCCD(
                v_t0, f0_t0, f1_t0, f2_t0,
                v_t1, f0_t1, f1_t1, f2_t1,
                err, mc, toi, tolerance, tmax, tmaxiter, outtolerance
            );
           if (a) {
               mesh.data(v).color = Vec(1, 0, 0);
               tios.push_back(toi);
               vi[v.idx()][0] *= toi;
               vi[v.idx()][1] *= toi;
               vi[v.idx()][2] *= toi;
               auto di = mesh.data(v).C * vi[v.idx()];
               mesh.point(v) = MyMesh::Point(di[0], di[1], di[2]);
           }
           else
           {
               //mesh.data(v).color = Vec(0, 0, 0);
           }
            //break;
        }
        //break;
    }
}

void MyViewer::collisonTest(std::vector<Eigen::Vector4d> v)
{
    index.clear();
    // createL_smooot(mesh);
    Eigen::MatrixXd V, VS;
    Eigen::MatrixXi F;
    saveMeshToEigen(mesh, V);
    saveMeshToEigen(smooth, VS);
    saveMeshFaceToEigen(smooth, F);
    igl::Timer timer;
    timer.start();
   
    //VS /= 5;
    mcl::BVHTree<double, 3> tree;
    tree.options.box_eta = std::numeric_limits<float>::epsilon();
    ;
    std::vector<std::tuple<Eigen::Vector4i, int, double>> contacts;  // CCD results: (collision info, type, TOI)
    std::set<std::pair<int, int>> discrete;
    tbb::concurrent_unordered_set<Eigen::Vector4i> co;
    tree.update(V, VS, F); // creates or updates BVH
    tree.options.threaded = false;
    tree.append_pair = [&](const Eigen::Vector4i& sten, int type, const double& toi)->void
    {
        double s = toi *10;
        if ( s <= 1)
        {
            contacts.emplace_back(sten, type, toi);
            
            //co.emplace(sten);
        }           
    };
    tree.append_discrete = [&](int p0, int p1)->bool
    {
        discrete.emplace(p0, p1); // tri-tri or edge-edge
        return false; // return true to stop traversing
    };
    tree.traverse(V, VS, F); // perform collision detection
    timer.stop();
    double tt = timer.getElapsedTimeInSec();
    int Tsize = contacts.size();
    int Set = co.size();
    std::vector<Eigen::Vector4i> tios;
    for (const auto& contact : contacts) {
        double fa = std::get<2>(contact);
        
        Eigen::Vector4i sten = std::get<0>(contact);
        tios.push_back(sten);
        int v_idx = sten[0];
        index.push_back(v_idx);
    }
    

    for (const auto& contact : contacts) {
        Eigen::Vector4i sten = std::get<0>(contact);  // Collision info
        double fa = std::get<2>(contact);
        int v_idx = sten[0];  // Index of the collided vertex



        //fa = 0.95;
        // Set the color of the collided vertex to red
        if (v_idx < mesh.n_vertices()) {
            MyMesh::VertexHandle vh = mesh.vertex_handle(v_idx);
            //mesh.set_color(vh, MyMesh::Color(255, 0, 0));  // Set color to red (RGB)
            mesh.data(vh).color = Vec(1, 0, 0);
            fa =1.0-fa;
            //TODO : Work on the right factor
            //v[vh.idx()][0] *= fa;
            //v[vh.idx()][1] *= fa;
            //v[vh.idx()][2] *= fa;

            auto d =mesh.data(vh).C * v[vh.idx()];
            mesh.point(vh) = MyMesh::Point(d[0], d[1], d[2]);
            for (auto vi : mesh.vv_range(vh)) {
                fa = 0.99;
               //v[vi.idx()][0] *= fa;
              //v[vi.idx()][1] *= fa;
              //v[vi.idx()][2] *= fa;
                auto di = mesh.data(vi).C * v[vi.idx()];
                mesh.point(vi) = MyMesh::Point(di[0], di[1], di[2]);
            }


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