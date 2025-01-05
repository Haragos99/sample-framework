#include "Skinning.h"
#include <Eigen/Eigen>

class BoneHeat : public Skinning {

	BoneHeat() = default;
	void execute(std::shared_ptr<BaseMesh> basemesh, std::vector<Bone>& bones) override;
	~BoneHeat() = default;
private:
	 
	Eigen::SparseMatrix<double> createLaplaceMatrix(MyMesh& mesh);
	Eigen::SparseMatrix<double> createDiagolaleMatrix(MyMesh& mesh);
	void calculateOptimalWeights(Eigen::SparseMatrix<double>& Diagolnal, Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>>& solver, MyMesh& mesh,int nbone);

	float epsilon;
};