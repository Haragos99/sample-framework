#pragma once
#include "src/skeleton/Bone.h"
#include "src/BaseMesh.h"
#include <QObject>
/*
* TODO: Create a Skinning only for LBS and DQS and create Skinningstrategy for the other which they use for animation
*/
class Skinning : public QObject
{
	Q_OBJECT
public:
	
	Skinning() = default;// Todo: Use Depedency insted asociason
	void calculateSkinning(MyMesh& mesh, std::vector<Bone>& bones);
	virtual void execute(std::shared_ptr<BaseMesh> basemesh, std::vector<Bone>& bones);
	const std::vector<std::shared_ptr<Object3D>>& getDebugMeshes() const {
		return debugMeshes;
	}
	virtual void animatemesh(std::shared_ptr<BaseMesh> basemesh, std::vector<Bone>& bones, bool inv = false);
	virtual ~Skinning();
signals:
	void progressUpdated(int value);
	void startProgress(QString message);
	void displayMessage(QString message);
	void endProgress();
protected:

	double distance(Vec p, Vec p1);
	void clean(MyMesh& mesh, std::vector<Bone>& bones);
	void addColor(std::shared_ptr<BaseMesh> basemesh, std::vector<Bone>& bones);
	std::vector<std::shared_ptr<Object3D>> debugMeshes;

};