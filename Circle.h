#pragma once
#include <QGLViewer/qglviewer.h>
using qglviewer::Vec;

// May be it is not important
//TODO: Finsih Implemate this logig later
class CursolCircle {
private:
	Vec position;
	Vec normal;
	int numSegments;
	float radius;
public:
	CursolCircle() = default;
	void setNormal();
	void setPosition();
	void setRadius();
	void draw();


};