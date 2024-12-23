#pragma once
#include "Skelton.h"
#include "BaseMesh.h"
class Skinning
{
public:
	
	Skinning(Skelton& skelton_, MyMesh& mesh_);// Todo: Use Depedency insted asociason
	void calculateSkinning();
	virtual void execute(BaseMesh& basemesh, Skelton& skelton);
	const std::vector<BaseMesh>& getDebugMeshes() const {
		return debugMeshes;
	}
	virtual ~Skinning();
protected:

	MyMesh& mesh;
	Skelton& skelton;//rethink
	double distance(Vec p, Vec p1);
	void clean(MyMesh::VertexHandle& v);
	std::vector<BaseMesh> debugMeshes;
};