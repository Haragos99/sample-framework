#include "../MyViewer.h"
#include "BoneHeat.h"

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



