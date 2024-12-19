#pragma once
#include "Object3D.h"

class BaseMesh : public Object3D
{
public:
	BaseMesh(std::string filename);
	virtual void draw(Visualization vis) override;


	virtual~BaseMesh();


private:
	MyMesh mesh;

};

