#pragma once
#include "Skelton.h"
#include "BaseMesh.h"

/*
* TODO: Create a Skinning only for LBS and DQS and create Skinningstrategy for the other which they use for animation
*/
class Skinning
{
public:
	
	Skinning() = default;// Todo: Use Depedency insted asociason
	void calculateSkinning(MyMesh& mesh, Skelton& skelton);
	virtual void execute(BaseMesh& basemesh, Skelton& skelton);
	const std::vector<std::shared_ptr<Object3D>>& getDebugMeshes() const {
		return debugMeshes;
	}
	virtual void animatemesh(BaseMesh& basemesh, Skelton& skelton);
	virtual ~Skinning();
protected:

	double distance(Vec p, Vec p1);
	void clean(MyMesh& mesh, Skelton& skelton);
	std::vector<std::shared_ptr<Object3D>> debugMeshes;

};