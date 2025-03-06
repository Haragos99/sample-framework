#include "DualQuaternion.h"


DualQuaternion::DualQuaternion(const MyQuaternion& q0, const MyQuaternion& qe)
{
	rotation = q0;
	translation = qe;
}


DualQuaternion::DualQuaternion(const MyQuaternion& q, const Vec& t)
{
	DualQuaternion res = dualquatfrom(q, t);
	*this = res;
}



DualQuaternion::DualQuaternion(Mat4& matrix)
{
	MyQuaternion q(matrix);
	Vec t(matrix[3][0], matrix[3][1], matrix[3][2]);
	DualQuaternion res = dualquatfrom(q, t);
	*this = res;
}


DualQuaternion DualQuaternion::dualquatfrom(const MyQuaternion& q, const Vec& t) const
{
	float w = -0.5f * (t.x * q.i() + t.y * q.j() + t.z * q.k());
	float i = 0.5f * (t.x * q.w() + t.y * q.k() - t.z * q.j());
	float j = 0.5f * (-t.x * q.k() + t.y * q.w() + t.z * q.i());
	float k = 0.5f * (t.x * q.j() - t.y * q.i() + t.z * q.w());

	return DualQuaternion(q, MyQuaternion(w, i, j, k));
}



Quaternion DualQuaternion::addition(Quaternion a, Quaternion&& b)
{
	float x = a[0] + b[0];
	float y = a[1] + b[1];
	float z = a[2] + b[2];
	float w = a[3] + b[3];


	return Quaternion(x,y,z,w);
}


Quaternion DualQuaternion::Quaternionscalar(Quaternion q, float scalar)
{
	float x = q[0] * scalar;
	float y = q[1] * scalar;
	float z = q[2] * scalar;
	float w = q[3] * scalar;


	return Quaternion(x, y, z, w);
}


Vec DualQuaternion::rotate(const Vec& v) const
{
	MyQuaternion tmp = rotation;
	tmp.normalize();
	return tmp.rotate(v);
}


void DualQuaternion::animate()
{

}


Vec DualQuaternion::transform(const Vec& p) const
{
	// As the dual quaternions may be the results from a
	// linear blending we have to normalize it :

	float norm = rotation.norm();
	MyQuaternion qblend_0 = rotation / norm;
	MyQuaternion qblend_e = translation / norm;

	// Translation from the normalized dual quaternion equals :
	// 2.f * qblend_e * conjugate(qblend_0)
	Vec v0 = qblend_0.get_vec_part();
	Vec ve = qblend_e.get_vec_part();
	Vec trans = (ve * qblend_0.w() - v0 * qblend_e.w() + cross(v0 ,ve)) * 2.f;


	auto d = (2.f * (qblend_e * qblend_0.conjugate()));

	// Rotate
	return qblend_0.rotate(p - trans) + trans ;
}

