#include "Skinning.h"
#include <Eigen/Eigen>

class BoneHeat : public Skinning {


	void execute(BaseMesh& basemesh, Skelton& skelton) override;
private:
	 
	Eigen::SparseMatrix<double> createLaplaceMatrix(MyMesh& mesh);
	Eigen::SparseMatrix<double> createDiagolaleMatrix(MyMesh& mesh);
	void calculateOptimalWeights(Eigen::SparseMatrix<double>& Diagolnal, Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>>& solver, MyMesh& mesh,int nbone);

	float epsilon;
};