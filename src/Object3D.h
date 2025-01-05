#pragma once
#include <string>
#include "Mesh.h"
#include "Keyframe.h"
#include "Visualization.h"

class Object3D {
public:
	explicit Object3D(std::string _filename): filename(_filename){}
	explicit Object3D() = default;
	virtual void draw(Visualization& vis)=0;
	virtual void drawWithNames(Visualization& vis) const = 0;
	virtual Vec postSelection(const int p) = 0;
	virtual void movement(int selected, const Vector& position) = 0;
	virtual void rotate(int selected, Vec angel) = 0;
	virtual void scale(float scale) = 0;
	virtual void animate(float time) = 0;
	virtual ~Object3D(){}
protected:
	int id;
	std::string filename;
	std::vector<Keyframe> keyframes;

};