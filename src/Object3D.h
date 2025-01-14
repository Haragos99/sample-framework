#pragma once
#include <string>
#include "Mesh.h"
#include "Keyframe.h"
#include "Visualization.h"


class Object3D {
public:
	explicit Object3D(std::string _filename): filename(_filename){}
	explicit Object3D() = default;
	virtual void draw(Vis::Visualization& vis)=0;
	virtual void drawWithNames(Vis::Visualization& vis) const = 0;
	virtual Vec postSelection(const int p) = 0;
	virtual void movement(int selected, const Vector& position) = 0;
	virtual void rotate(int selected, Vec angel) = 0;
	virtual void scale(float scale) = 0;
	virtual void animate(float time) = 0;
	virtual void setCameraFocus(Vector& min, Vector& max) = 0;
	virtual void addKeyframes(int selected,float timeline) = 0;
	virtual void reset() = 0;
	virtual void datainfo() = 0;
	virtual ~Object3D(){}
protected:
	static int id;
	std::string filename;
	std::vector<Keyframe> keyframes;

};