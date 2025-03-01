#pragma once
#include "Object3D.h"

class BaseMesh : public Object3D
{
public:
	BaseMesh(std::string filename);
	BaseMesh(MyMesh& _mesh);
	virtual void draw(Vis::Visualization& vis) override;
	virtual void drawWithNames(Vis::Visualization& vis) const override;
	virtual Vec postSelection(const int p)  override;
	virtual void movement(int selected, const Vector& position) override;
	virtual void rotate(int selected, Vec angel) override;
	virtual void animate(float time) override;
	virtual void scale(float scale)override;
	virtual MyMesh& getMesh();
	virtual void setCameraFocus(Vector& min, Vector& max) override;
	void addKeyframes(int selected,float timeline) override;
	void reset()override;
	void datainfo() override;
	bool open();
	void grab(int selected, const Vec& position);
	double distance(Vec p, Vec p1);
	void addcolor(Vec& color);
	void setMesh(MyMesh& _mesh);
	void setFilename(std::string _filename);
	double voronoiWeight(MyMesh::HalfedgeHandle in_he);
	void updateMesh();
	void fairMesh();
	virtual~BaseMesh();


private:
	void updateMeanMinMax();
	void updateVertexNormals();
	void updateMeanCurvature();
	MyMesh mesh;
	std::vector<Vec> colors;

};

