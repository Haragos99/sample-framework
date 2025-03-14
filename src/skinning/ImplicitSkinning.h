#pragma once
#include "Skinning.h"
#include "src/HRBF.h"
#include "src/PoissonSampleGenerator.h"

class ImplicitSkinning :public Skinning {
	Q_OBJECT
public:
	ImplicitSkinning() = default;
	void execute(std::shared_ptr<BaseMesh> basemesh, std::vector<Bone>& bones);
	~ImplicitSkinning() = default;
private:
	PoissonSampleGenerator poissongenerator;
	std::vector<std::shared_ptr<HRBF>> implicitspaces;
	void seperateMesh(std::shared_ptr<BaseMesh>, int nbones);
	void generatesampels();
	std::vector<std::shared_ptr<BaseMesh>> separetmeshes;
	std::vector<std::vector<MyMesh::Point>> seprateSampels;
	std::vector<std::vector<MyMesh::Normal>> normalsofsampels;
};


