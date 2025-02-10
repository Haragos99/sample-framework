#pragma once
#include <QGLViewer/quaternion.h>
#include "../Matrix4.h"

using qglviewer::Quaternion;
using qglviewer::Vec;

class DualQuaternion {
private:
	Quaternion rotation;
	Quaternion translation;


	DualQuaternion dualquatfrom(const Quaternion& q, const Vec& t) const;

public:
	DualQuaternion(const Quaternion& q0, const Quaternion& qe);
	DualQuaternion(const Quaternion& q, const Vec& t);

	DualQuaternion(Mat4& matrix);
	
	void animate();

	Quaternion getRotation() const { return rotation; }

	Quaternion getTranslation() const { return translation; }

	Quaternion addition(Quaternion a, Quaternion&& b);


	Vec rotate(const Vec& v) const;


	Quaternion Quaternionscalar(Quaternion q, float scalar);

	DualQuaternion operator+(const DualQuaternion& dq) 
	{
		return DualQuaternion(addition(rotation, dq.getRotation()), addition(translation, dq.getTranslation()));
	}


	DualQuaternion operator*(float scalar) 
	{
		return DualQuaternion(Quaternionscalar(rotation, scalar), Quaternionscalar(translation, scalar));
	}

	static DualQuaternion identity()
	{
		return DualQuaternion(Quaternion(0.f, 0.f, 0.f, 1.f),
			Vec(0.f, 0.f, 0.f));
	}


};