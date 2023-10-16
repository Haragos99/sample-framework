#include "MyViewer.h"


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
        mesh.data(v).weigh.resize(b.size(), 0.0);
        mesh.data(v).distance.clear();
        mesh.data(v).distance.resize(b.size(), min_val);

        // az aktu�lis pont
        Vec d = Vec(mesh.point(v)[0], mesh.point(v)[1], mesh.point(v)[2]);
        int indexof = 0;
        for (int i = 0; i < b.size(); i++)
        {
            double f = std::numeric_limits<double>::infinity(); ;
            for (int j = 0; j < b[i].points.size(); j++)
            {
                // printf("%d %d", i, j);
                double t = distance(b[i].points[j], d);

                // legk�zelebbi s�ly
                if (t < min_val)
                {
                    min_val = t;
                    indexof = i;
                }
                // csonton l�v� legk�zelebbi t�vols�g
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
    // az aktu�los pont folyamatosan n�velj�k
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

        // a D m�trix �tloja felt�lt�se
        D.coeffRef(v.idx(), v.idx()) = er;
    }
    D.makeCompressed();
    D *= epsilon;
    Eigen::SparseMatrix<double> O = -L + D;
    Eigen::SimplicialLDLT< Eigen::SparseMatrix<double> > solver;
    //Eigen::SparseLU< Eigen::SparseMatrix<double> > solver;
    solver.compute(O); // M inverz m�trix�nak (val�j�ban: "LU felbont�s�nak") kisz�m�t�sa


    for (int k = 0; k < b.size(); k++)
    {
        Eigen::VectorXd w1(n);
        w1 = Eigen::VectorXd::Zero(n); // Felt�ltj�k 0-kkal
        for (auto v : mesh.vertices()) {
            w1[v.idx()] = mesh.data(v).weigh[k];
        }

        Eigen::VectorXd Sw1(n);

        Sw1 = D * w1;


        if (solver.info() == Eigen::Success) { // Tesztelj�k, hogy siker�lt-e invert�lni

            Eigen::VectorXd x = solver.solve(Sw1); // Megoldjuk az M*x = v egyenletrendszert

            if (solver.info() == Eigen::Success) { // Tesztelj�k, hogy siker�lt-e megoldani

                for (auto v : mesh.vertices()) {

                    mesh.data(v).weigh[k] = x[v.idx()];

                    //std::cerr << "Ez egy sz�veg, ez egy sz�m: " << x[v.idx()] << std::endl;
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
        for (int i = 0; i < b.size(); i++) {
            sum += mesh.data(v).weigh[i];
        }
        if (abs(sum - 1.0) > 1E-10) {
            displayMessage("SUM != 1 !!! @ vertex " + v.idx());
        }
    }
    omerr() << "Bone heat finish:" << QDateTime::currentDateTime().toString("hh:mm:ss:zzz").toStdString() << std::endl;
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


        for (auto vi : mesh.vv_range(v)) { // v vertex-szel szomsz�dos vertexek


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

