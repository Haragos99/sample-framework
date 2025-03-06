#pragma once
#include <QGLViewer/quaternion.h>
#include "../MyQuaternion.hpp"

using qglviewer::Quaternion;
using qglviewer::Vec;

class DualQuaternion {
private:
	MyQuaternion rotation;
	MyQuaternion translation;


	DualQuaternion dualquatfrom(const MyQuaternion& q, const Vec& t) const;

public:
	DualQuaternion(const MyQuaternion& q0, const MyQuaternion& qe);
	DualQuaternion(const MyQuaternion& q, const Vec& t);

	DualQuaternion(Mat4& matrix);
	
	void animate();

	MyQuaternion getRotation() const { return rotation; }

	MyQuaternion getTranslation() const { return translation; }

	Quaternion addition(Quaternion a, Quaternion&& b);


	Vec rotate(const Vec& v) const;


	Quaternion Quaternionscalar(Quaternion q, float scalar);

	DualQuaternion operator+(const DualQuaternion& dq) 
	{
		return DualQuaternion(rotation + dq.getRotation(), translation + dq.getTranslation());
	}


	DualQuaternion operator*(float scalar) 
	{
		return DualQuaternion(rotation * scalar, translation * scalar);
	}

	static DualQuaternion identity()
	{
		return DualQuaternion(MyQuaternion(1.f, 0.f, 0.f, 0.f),
			Vec(0.f, 0.f, 0.f));
	}

	Vec transform(const Vec& p) const;

};