#pragma once
#include <string>
#include "Mesh.h"
#include "Keyframe.h"
#include "Visualization.h"

class Object3D {
public:
	explicit Object3D(std::string filename);
	virtual ~Object3D();
	virtual void draw(Visualization vis);
	virtual void drawWithNames() const = 0;
	virtual void postSelection(const int p) = 0;
	virtual void movement(int selected, const Vector& position) = 0;
	virtual void rotate(int selected, Vec angel) = 0;
	

protected:
	int id;
	std::string filename;
	std::vector<Keyframe> keyframes;
};