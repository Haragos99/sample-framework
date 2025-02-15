#include "DualQuaternion.h"


DualQuaternion::DualQuaternion(const Quaternion& q0, const Quaternion& qe)
{
	rotation = q0;
	translation = qe;
}


DualQuaternion::DualQuaternion(const Quaternion& q, const Vec& t)
{
	DualQuaternion res = dualquatfrom(q, t);
	*this = res;
}



DualQuaternion::DualQuaternion(Mat4& matrix)
{
	double m[3][3] = {
	{ matrix[0][0], matrix[0][1], matrix[0][2] },
	{ matrix[1][0], matrix[1][1], matrix[1][2] },
	{ matrix[2][0], matrix[2][1], matrix[2][2] }
	};
	Quaternion q;
	q.setFromRotationMatrix(m);
	Vec t(matrix[3][0], matrix[2][1], matrix[3][2]);
	DualQuaternion res = dualquatfrom(q, t);
	*this = res;
}


DualQuaternion DualQuaternion::dualquatfrom(const Quaternion& q, const Vec& t) const
{
	float w = -0.5f * (t.x *  q[0] + t.y  *  q[1] + t.z  * q[2]);
	float i = 0.5f * (t.x  *  q[3] + t.y  *  q[2] - t.z  * q[1]);
	float j = 0.5f * (-t.x *  q[2] + t.y  *  q[3] + t.z  * q[0]);
	float k = 0.5f * (t.x  *  q[1] - t.y  *  q[1] + t.z  * q[3]);

	return DualQuaternion(q,Quaternion(i,j,k,w));
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
	Quaternion tmp = rotation;
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

	Quaternion qblend_0 = rotation.normalized();
	Quaternion qblend_e = translation.normalized();

	// Translation from the normalized dual quaternion equals :
	// 2.f * qblend_e * conjugate(qblend_0)
	Vec v0 = Vec(qblend_0[0], qblend_0[1], qblend_0[2]);
	Vec ve = Vec(qblend_e[0], qblend_e[1], qblend_e[2]);
	Vec trans = (ve * qblend_0[3] - v0 * qblend_e[3] + (v0 ^ ve)) * 2.f;

	// Rotate
	return qblend_0.rotate(p) + trans;
}

