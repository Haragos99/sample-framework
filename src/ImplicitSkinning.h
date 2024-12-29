#pragma once
#include "Skinning.h"
#include "HRBF.h"
#include "PoissonSampleGenerator.h"

class ImplicitSkinning :public Skinning {
public:
	ImplicitSkinning() = default;
	void execute(BaseMesh& basemesh, Skelton& skelton);
	~ImplicitSkinning() = default;
private:
	PoissonSampleGenerator poissongenerator;
	std::vector<std::shared_ptr<HRBF>> implicitspaces;
	void seperateMesh(BaseMesh& basemesh, int nbones);
	void generatesampels();
	std::vector<std::shared_ptr<BaseMesh>> separetmeshes;
	std::vector<std::vector<MyMesh::Point>> seprateSampels;
	std::vector<std::vector<MyMesh::Normal>> normalsofsampels;
};


