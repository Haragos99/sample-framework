#pragma once
#include "Skelton.h"
#include "BaseMesh.h"
class Skinning
{
public:
	
	Skinning() = default;// Todo: Use Depedency insted asociason
	void calculateSkinning(MyMesh& mesh, Skelton& skelton);
	virtual void execute(BaseMesh& basemesh, Skelton& skelton);
	const std::vector<BaseMesh>& getDebugMeshes() const {
		return debugMeshes;
	}
	virtual ~Skinning();
protected:

	double distance(Vec p, Vec p1);
	void clean(MyMesh& mesh, Skelton& skelton);
	std::vector<BaseMesh> debugMeshes;
};