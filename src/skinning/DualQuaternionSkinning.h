#pragma once
#include <QGLViewer/quaternion.h>

using qglviewer::Quaternion;
using qglviewer::Vec;

class DualQuaternionSkinning {
private:
	Quaternion rotation;
	Quaternion translation;


	DualQuaternionSkinning dualquatfrom(const Quaternion& q, const Vec& t) const;

public:
	DualQuaternionSkinning(const Quaternion& q0, const Quaternion& qe);
	DualQuaternionSkinning(const Quaternion& q, const Vec& t);
	

	void animate();

	Quaternion getRotation() const { return rotation; }

	Quaternion getTranslation() const { return translation; }

	Quaternion addition(Quaternion a, Quaternion&& b);


	Vec rotate(const Vec& v) const;


	Quaternion Quaternionscalar(Quaternion q, float scalar);

	DualQuaternionSkinning operator+(const DualQuaternionSkinning& dq) 
	{
		return DualQuaternionSkinning(addition(rotation, dq.getRotation()), addition(translation, dq.getTranslation()));
	}


	DualQuaternionSkinning operator*(float scalar) 
	{
		return DualQuaternionSkinning(Quaternionscalar(rotation, scalar), Quaternionscalar(translation, scalar));
	}

	static DualQuaternionSkinning identity()
	{
		return DualQuaternionSkinning(Quaternion(0.f, 0.f, 0.f, 1.f),
			Vec(0.f, 0.f, 0.f));
	}


};