#pragma once
#include "BaseMesh.h"


//TODO: Finsih Implemate this logig later

class Sculpt {
private:
	std::shared_ptr<BaseMesh> basemesh;
	float radius;
	Vis::SculptType selectedType;
public:
	Sculpt(std::shared_ptr<BaseMesh> _basemesh) : basemesh(_basemesh), radius(0.3), selectedType(Vis::SculptType::GRAB) {}

	void grab(int selected, const Vec& position);

	void smooth(int selected, const Vec& position);

	void addRaduis(float r) { radius += r; }

	void minusRaduis(float r) { radius -= r; }

	void excecut(int selected, const Vec& position);

	Vec getNormal();

	double distance(Vec p, Vec p1);

};