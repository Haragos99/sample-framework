﻿#include "MyViewer.h"
#include "BoneHeat.h"

void MyViewer::weigh()
{
    visualization = Visualization::WEIGH;
    model_type = ModelType::SKELTON;
    mehet = true;
    isweight = true;
    for (auto v : mesh.vertices())
    {
        double min_val = std::numeric_limits<double>::infinity();
        mesh.data(v).weigh.clear();
        mesh.data(v).weigh.resize(skel.getSize(), 0.0);
        mesh.data(v).distance.clear();
        mesh.data(v).distance.resize(skel.getSize(), min_val);

        // az aktuális pont
        Vec d = Vec(mesh.point(v)[0], mesh.point(v)[1], mesh.point(v)[2]);
        int indexof = 0;
        for (int i = 0; i < skel.getSize(); i++)
        {
            double f = std::numeric_limits<double>::infinity(); ;
            for (int j = 0; j < skel.bones[i].points.size(); j++)
            {
                // printf("%d %d", i, j);
                double t = distance(skel.bones[i].points[j], d);

                // legközelebbi súly
                if (t < min_val)
                {
                    min_val = t;
                    indexof = i;
                }
                // csonton lévő legközelebbi távolság
                if (t < f)
                {
                    f = t;
                }

            }
            mesh.data(v).distance[i] = f;
        }
        mesh.data(v).weigh[indexof] = 1;
        mesh.data(v).idx_of_closest_bone = indexof;
    }
}

void MyViewer::Smooth()
{
    omerr() << "Bone heat start: " << QDateTime::currentDateTime().toString("hh:mm:ss:zzz").toStdString() << std::endl ;
    int n = mesh.n_vertices();
    // az aktuálos pont folyamatosan növeljük
    Eigen::SparseMatrix<double> L;
    createL(L);

    Eigen::SparseMatrix<double> D(n, n);
    Eigen::VectorXd nnz = Eigen::VectorXd::Zero(n);
    for (auto v : mesh.vertices()) {
nnz(v.idx()) = 1;
    }
    D.makeCompressed();
    D.reserve(nnz);
    for (auto v : mesh.vertices()) {
        double er = 1.0 / pow(mesh.data(v).distance[mesh.data(v).idx_of_closest_bone], 2);

        // a D mátrix átloja feltöltése
        D.coeffRef(v.idx(), v.idx()) = er;
    }
    D.makeCompressed();
    D *= epsilon;
    Eigen::SparseMatrix<double> O = -L + D;
    Eigen::SimplicialLDLT< Eigen::SparseMatrix<double> > solver;
    //Eigen::SparseLU< Eigen::SparseMatrix<double> > solver;
    solver.compute(O); // M inverz mátrixának (valójában: "LU felbontásának") kiszámítása


    for (int k = 0; k < skel.getSize(); k++)
    {
        Eigen::VectorXd w1(n);
        w1 = Eigen::VectorXd::Zero(n); // Feltöltjük 0-kkal
        for (auto v : mesh.vertices()) {
            w1[v.idx()] = mesh.data(v).weigh[k];
        }

        Eigen::VectorXd Sw1(n);

        Sw1 = D * w1;


        if (solver.info() == Eigen::Success) { // Teszteljük, hogy sikerült-e invertálni

            Eigen::VectorXd x = solver.solve(Sw1); // Megoldjuk az M*x = v egyenletrendszert

            if (solver.info() == Eigen::Success) { // Teszteljük, hogy sikerült-e megoldani

                for (auto v : mesh.vertices()) {

                    mesh.data(v).weigh[k] = x[v.idx()];

                    //std::cerr << "Ez egy szöveg, ez egy szám: " << x[v.idx()] << std::endl;
                }

            }
            else {
                emit displayMessage("System solve FAIL!!!");
            }
        }
        else {
            emit displayMessage("Matrix inversion FAIL!!!");
        }

    }

    for (auto v : mesh.vertices()) {
        double sum = 0.0;
        for (int i = 0; i < skel.getSize(); i++) {
            sum += mesh.data(v).weigh[i];
        }
        if (abs(sum - 1.0) > 1E-10) {
            displayMessage("SUM != 1 !!! @ vertex " + v.idx());
        }
    }
    omerr() << "Bone heat finish:" << QDateTime::currentDateTime().toString("hh:mm:ss:zzz").toStdString() << std::endl;
}

Eigen::SparseMatrix<double>  BoneHeat::createDiagolaleMatrix(MyMesh& mesh)
{
    int n = mesh.n_vertices();
    epsilon = 0.001;

    Eigen::SparseMatrix<double> Diagolal(n, n);
    Eigen::VectorXd nnz = Eigen::VectorXd::Zero(n);
    for (auto v : mesh.vertices()) {
        nnz(v.idx()) = 1;
    }
    Diagolal.makeCompressed();
    Diagolal.reserve(nnz);
    for (auto v : mesh.vertices()) {
        double er = 1.0 / pow(mesh.data(v).distance[mesh.data(v).idx_of_closest_bone], 2);

        // a D mátrix átloja feltöltése
        Diagolal.coeffRef(v.idx(), v.idx()) = er;
    }
    Diagolal.makeCompressed();
    Diagolal *= epsilon;

    return Diagolal;
}

void BoneHeat::execute(std::shared_ptr<BaseMesh> basemesh, std::vector<Bone>& bones)
{
    MyMesh& mesh = basemesh->getMesh();
    calculateSkinning(mesh, bones);
    addColor(basemesh, bones);
    auto LaplaceM = createLaplaceMatrix(mesh);
    auto DiagolalM = createDiagolaleMatrix(mesh);
    Eigen::SparseMatrix<double> O = -LaplaceM + DiagolalM;
    Eigen::SimplicialLDLT< Eigen::SparseMatrix<double>> solver;
    solver.compute(O); // For Inverz Matrix 
    if (solver.info() == Eigen::Success) 
    {
        // solve this equliton (L + 𝜆D) * w𝑘 = 𝜆D * ~wk
        calculateOptimalWeights(DiagolalM, solver, mesh, bones.size());
    }
}


void BoneHeat::calculateOptimalWeights(Eigen::SparseMatrix<double>& Diagolnal, 
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>>& solver,MyMesh& mesh, int nbone)
{
    int n = mesh.n_vertices();
    for (int i = 0; i < nbone; i++)
    {
        Eigen::VectorXd baseweigh(n);
        baseweigh = Eigen::VectorXd::Zero(n); // init with zeros
        for (auto v : mesh.vertices()) {
            baseweigh[v.idx()] = mesh.data(v).weigh[i];
        }
        Eigen::VectorXd Dbaseweigh(n);
        Dbaseweigh = Diagolnal * baseweigh;
        Eigen::VectorXd optimalweigh = solver.solve(Dbaseweigh);
        if (solver.info() == Eigen::Success) { 

            for (auto v : mesh.vertices()) {
                mesh.data(v).weigh[i] = optimalweigh[v.idx()];
            }
        }  
    }
}



Eigen::SparseMatrix<double> BoneHeat::createLaplaceMatrix(MyMesh& mesh)
{
    Eigen::SparseMatrix<double> Laplace;

    int n = mesh.n_vertices();
    if (Laplace.rows() < n || Laplace.cols() < n) {
        Laplace.resize(n, n);
    }

    Eigen::VectorXd nnz = Eigen::VectorXd::Zero(n);
    for (auto v : mesh.vertices()) {
        nnz(v.idx()) = 1 + mesh.valence(v);
    }
    Laplace.makeCompressed();
    Laplace.reserve(nnz);

    int i = 0;
    for (auto v : mesh.vertices()) {
        for (auto vi : mesh.vv_range(v)) { // v vertex-szel szomszédos vertexek


            auto eij = mesh.find_halfedge(v, vi);
            auto m1 = mesh.next_halfedge_handle(eij);
            double theta1 = mesh.calc_sector_angle(m1);

            auto eji = mesh.find_halfedge(vi, v);
            auto m2 = mesh.next_halfedge_handle(eji);
            double theta2 = mesh.calc_sector_angle(m2);

            double result = (tan(M_PI_2 - theta1) + tan(M_PI_2 - theta2)) / 2;
            if (i == vi.idx()) continue;
            Laplace.coeffRef(i, vi.idx()) = result;
        }
        i++;
    }
    for (int i = 0; i < n; i++)
    {

        Laplace.coeffRef(i, i) = -Laplace.row(i).sum();
    }
    Laplace.makeCompressed();

    return Laplace;

}

void MyViewer::createL(Eigen::SparseMatrix<double>& L)
{

    int n = mesh.n_vertices();
    if (L.rows() < n || L.cols() < n) {
        L.resize(n, n);
    }

    Eigen::VectorXd nnz = Eigen::VectorXd::Zero(n);
    for (auto v : mesh.vertices()) {
        nnz(v.idx()) = 1 + mesh.valence(v);
    }
    L.makeCompressed();
    L.reserve(nnz);

    int i = 0;
    for (auto v : mesh.vertices()) {


        for (auto vi : mesh.vv_range(v)) { // v vertex-szel szomszédos vertexek


            auto eij = mesh.find_halfedge(v, vi);
            auto m1 = mesh.next_halfedge_handle(eij);
            double theta1 = mesh.calc_sector_angle(m1);


            auto eji = mesh.find_halfedge(vi, v);
            auto m2 = mesh.next_halfedge_handle(eji);
            double theta2 = mesh.calc_sector_angle(m2);

            double result = (tan(M_PI_2 - theta1) + tan(M_PI_2 - theta2)) / 2;
            if (i == vi.idx()) continue;
            L.coeffRef(i, vi.idx()) = result;
        }


        i++;
    }
    for (int i = 0; i < n; i++)
    {

        L.coeffRef(i, i) = -L.row(i).sum();
    }
    L.makeCompressed();

}

