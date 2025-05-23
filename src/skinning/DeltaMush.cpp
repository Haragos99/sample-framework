#include "DeltaMush.h"








std::shared_ptr<BaseMesh> DeltaMush::smoothMesh(std::shared_ptr<BaseMesh> basemesh)
{
    MyMesh& _mesh = basemesh->getMesh();
    double smootingfactor = 0.5;
    auto mesh_ = _mesh;
    for (int i = 0; i < 20; i++)
    {
        auto smooth = mesh_;
        for (auto v : mesh_.vertices()) {
            if (!_mesh.is_boundary(v))
            {
                Vec Avg;
                int n = 0;
                for (auto vi : mesh_.vv_range(v)) {
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
        }
    }
    smooth = mesh_;
    std::shared_ptr<BaseMesh> smooth = std::make_shared<BaseMesh>(mesh_);
    return smooth;
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


void DeltaMush::execute(std::shared_ptr<BaseMesh> basemesh, std::vector<Bone>& bones)
{
    debugMeshes.clear();
    delta.clear();
    lines = std::make_shared<DeltaLines>();
    calculateSkinning(basemesh->getMesh(), bones);
    addColor(basemesh, bones);
    Delta_Mush(basemesh);
}

void DeltaMush::Delta_Mush(std::shared_ptr<BaseMesh> basemesh)
{
    auto mesh = basemesh->getMesh();
    auto smooth_basemesh = smoothMesh(basemesh);
    auto smooth_mesh = smooth_basemesh->getMesh();
    smooth_mesh.request_face_normals();
    smooth_mesh.request_vertex_normals();
    smooth_mesh.update_normals();
    for (auto ve : smooth_mesh.vertices()) {

        Eigen::MatrixXd R;
        Eigen::FullPivLU< Eigen::MatrixXd> solver;
        Eigen::Vector4d p_vector;
        Eigen::Vector4d v_vector;
        p_vector << mesh.point(ve)[0], mesh.point(ve)[1], mesh.point(ve)[2], 1;
        R.resize(4, 4);
        MyMesh::Normal normal = smooth_mesh.normal(ve);
        MyMesh::HalfedgeHandle heh = *smooth_mesh.voh_iter(ve);
        auto ed = smooth_mesh.calc_edge_vector(heh);
        Vec t = Vec((ed - (ed | normal) * normal).normalize());
        Vec b = (t ^ Vec(normal)).unit();
        R = BuiledMatrix(normal, t, b, Vec(smooth_mesh.point(ve).data()));
        Eigen::MatrixXd I(4, 4);
        I.setIdentity();
        solver.compute(R);

        auto R_inv = solver.solve(I);

        v_vector = R_inv * p_vector;

        delta.push_back(v_vector);
        Vec start = Vec(smooth_mesh.point(ve).data());
        Vec end = Vec(mesh.point(ve).data());
        createDeltaline(start, end);

    }
    //debugMeshes.push_back(smooth_basemesh);
    debugMeshes.push_back(lines);
}

void DeltaMush::createDeltaline(Vec& start, Vec& end)
{
    Vec red = Vec(1, 0, 0);
    std::shared_ptr<Line> line = std::make_shared<Line>();
    line->setStart(start);
    line->setEnd(end);
    line->setcolor(red);
    lines->addLine(line);
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

void DeltaMush::animatemesh(std::shared_ptr<BaseMesh> basemesh, std::vector<Bone>& bones, bool inv)
{
    Skinning::animatemesh(basemesh,bones,inv);
    Delta_Mush_two(basemesh);
}


void DeltaMush::modifyDeltaLine(std::vector<Eigen::Vector4d> deltavector)
{
    lines->setDeltas(deltavector);
}

void DeltaMush::Delta_Mush_two(std::shared_ptr<BaseMesh> basemesh)
{
    auto v_d = setMushFactor(delta);
    modifyDeltaLine(v_d);
    auto& mesh = basemesh->getMesh();
    std::vector<Vec> smoothed;
    auto smooth_basemesh = smoothMesh(basemesh);
    auto smooth_mesh = smooth_basemesh->getMesh();
    smooth_mesh.request_face_normals();
    smooth_mesh.request_vertex_normals();
    smooth_mesh.update_normals();
    int size = smoothed.size();
    for (auto ve : mesh.vertices()) {
        Eigen::MatrixXd C(4, 4);
        Eigen::Vector4d p_vector;
        Eigen::Vector4d v_vector;
        p_vector << mesh.point(ve)[0], mesh.point(ve)[1], mesh.point(ve)[2], 1;
        MyMesh::Normal normal = smooth_mesh.normal(ve);
        MyMesh::HalfedgeHandle heh = *smooth_mesh.voh_iter(ve);
        auto ed = smooth_mesh.calc_edge_vector(heh);
        Vec t = Vec((ed - (ed | normal) * normal).normalize());
        Vec b = (t ^ Vec(normal)).unit();

        C = BuiledMatrix(normal, t, b, Vec(smooth_mesh.point(ve).data()));
        mesh.data(ve).C = C;
        auto d = C * v_d[ve.idx()];
        mesh.point(ve) = MyMesh::Point(d[0], d[1], d[2]);
    }
    //basemesh->setMesh(m);
}