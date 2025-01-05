#pragma once
#include"Object3D.h"

class Line : public Object3D {
public:
	Line() = default;
	virtual void draw(Vis::Visualization& vis) override;
	virtual void drawWithNames(Vis::Visualization& vis) const override;
	virtual Vec postSelection(const int p) override;
	virtual void movement(int selected, const Vector& position) override;
	virtual void rotate(int selected, Vec angel) override;
	virtual void animate(float time) override;
	virtual void scale(float scale) override;
	virtual void setCameraFocus(Vector& min, Vector& max) override;
	void setcolor(Vec& _color);
	void setStart(Vec& _start);
	void setEnd(Vec& _end);
	Vec getStart();
	Vec getEnd();
	~Line() {}
private:
	std::vector<Vec> points;
	Vec start;
	Vec end;
	Vec color;
	float width;
};

class DeltaLines : public Object3D {
public:
	void draw(Vis::Visualization& vis);
	void drawWithNames(Vis::Visualization& vis) const override;
	Vec postSelection(const int p) override;
	void movement(int selected, const Vector& position) override;
	void rotate(int selected, Vec angel) override;
	void animate(float time) override;
	void scale(float scale) override;
	void setCameraFocus(Vector& min, Vector& max) override;
	void setDeltas(std::vector<Eigen::Vector4d> deltas);
	void addLine(std::shared_ptr<Line> line);
	~DeltaLines() {}
private:
	std::vector< std::shared_ptr<Line>> lines;

};