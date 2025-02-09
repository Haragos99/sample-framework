#include "DualQuaternionSkinning.h"


DualQuaternionSkinning::DualQuaternionSkinning(const Quaternion& q0, const Quaternion& qe)
{
	rotation = q0;
	translation = qe;
}


DualQuaternionSkinning::DualQuaternionSkinning(const Quaternion& q, const Vec& t)
{
	DualQuaternionSkinning res = dualquatfrom(q, t);
	*this = res;
}


DualQuaternionSkinning DualQuaternionSkinning::dualquatfrom(const Quaternion& q, const Vec& t) const
{
	float w = -0.5f * (t.x * q[0] + t.y  * q[1] + t.z   * q[2]);
	float i = 0.5f * (t.x  *  q[3] + t.y *  q[2] - t.z  * q[1]);
	float j = 0.5f * (-t.x * q[2] + t.y  * q[3] + t.z   * q[0]);
	float k = 0.5f * (t.x  *  q[1] - t.y *  q[1] + t.z  * q[3]);

	return DualQuaternionSkinning(q,Quaternion(i,j,k,w));
}



Quaternion DualQuaternionSkinning::addition(Quaternion a, Quaternion&& b)
{
	float x = a[0] + b[0];
	float y = a[1] + b[1];
	float z = a[2] + b[2];
	float w = a[3] + b[3];


	return Quaternion(x,y,z,w);
}


Quaternion DualQuaternionSkinning::Quaternionscalar(Quaternion q, float scalar)
{
	float x = q[0] * scalar;
	float y = q[1] * scalar;
	float z = q[2] * scalar;
	float w = q[3] * scalar;


	return Quaternion(x, y, z, w);
}


Vec DualQuaternionSkinning::rotate(const Vec& v) const
{
	Quaternion tmp = rotation;
	tmp.normalize();
	return tmp.rotate(v);
}


void DualQuaternionSkinning::animate()
{

}

