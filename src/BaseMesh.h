#pragma once
#include "Object3D.h"

class BaseMesh : public Object3D
{
public:
	BaseMesh(std::string filename);
	BaseMesh(MyMesh& _mesh);
	virtual void draw(Visualization& vis) override;
	virtual void drawWithNames(Visualization& vis) const override;
	virtual Vec postSelection(const int p)  override;
	virtual void movement(int selected, const Vector& position) override;
	virtual void rotate(int selected, Vec angel) override;
	virtual void animate(float time) override;
	virtual void scale(float scale)override;
	virtual MyMesh& getMesh();
	bool open();
	void addcolor(Vec& color);
	void setMesh(MyMesh& _mesh);
	virtual~BaseMesh();


private:
	MyMesh mesh;
	std::vector<Vec> colors;

};

