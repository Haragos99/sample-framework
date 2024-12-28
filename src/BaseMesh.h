#pragma once
#include "Object3D.h"

class BaseMesh : public Object3D
{
public:
	BaseMesh(std::string filename);
	virtual void draw(Visualization& vis) override;
	virtual void drawWithNames(Visualization& vis) const override;
	virtual Vec postSelection(const int p)  override;
	virtual void movement(int selected, const Vector& position) override;
	virtual void rotate(int selected, Vec angel) override;
	virtual void animate(float time) override;
	MyMesh& getMesh();
	bool open();
	virtual~BaseMesh();


private:
	MyMesh mesh;
	std::vector<Vec> colors;

};

