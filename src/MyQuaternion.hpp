#include "Matrix4.h"



class MyQuaternion {
public:

    // -------------------------------------------------------------------------
    /// @name Constructors
    // -------------------------------------------------------------------------

    /// Default constructor : build a zero rotation.
    MyQuaternion()
    {
        coeff[0] = 1.f;
        coeff[1] = 0.f; coeff[2] = 0.f; coeff[3] = 0.f;
    }

    /// Copy constructor
    MyQuaternion(const MyQuaternion& q) {
        coeff[0] = q.w();
        coeff[1] = q.i(); coeff[2] = q.j(); coeff[3] = q.k();
    }

    /// directly fill the quaternion
    MyQuaternion(float w, float i, float j, float k) {
        coeff[0] = w;
        coeff[1] = i; coeff[2] = j; coeff[3] = k;
    }

    /// directly fill the quaternion vector and scalar part
    MyQuaternion(float w, const Vec& v) {
        coeff[0] = w;
        coeff[1] = v.x; coeff[2] = v.y; coeff[3] = v.z;
    }

    /// Construct the quaternion from the transformation matrix 't'
    /// Translation of 't' is ignored as quaternions can't represent it
    MyQuaternion(const Mat4& m)
    {
        // Compute trace of matrix 't'
        float T = 1 + m[0][0] + m[1][1] + m[2][2];

        float S, X, Y, Z, W;

        if (T > 0.00000001f) // to avoid large distortions!
        {
            S = sqrt(T) * 2.f;
            X = (m[1][2] - m[2][1]) / S;
            Y = (m[2][0] - m[0][2]) / S;
            Z = (m[0][1] - m[1][0]) / S;
            W = 0.25f * S;
        }
        else
        {
            if (m[0][0] > m[1][1] && m[0][0] > m[2][2])
            {
                // Column 0 :
                S = sqrt(1.0f + m[0][0] - m[1][1] - m[2][2]) * 2.f;
                X = 0.25f * S;
                Y = (m[0][1] + m[1][0]) / S;
                Z = (m[2][0] + m[0][2]) / S;
                W = (m[1][2] - m[2][1]) / S;
            }
            else if (m[1][1] > m[2][2])
            {
                // Column 1 :
                S = sqrt(1.0f + m[1][1] - m[0][0] - m[2][2]) * 2.f;
                X = (m[0][1] + m[1][0]) / S;
                Y = 0.25f * S;
                Z = (m[1][2] + m[2][1]) / S;
                W = (m[2][0] - m[0][2]) / S;
            }
            else
            {   // Column 2 :
                S = sqrt(1.0f + m[2][2] - m[0][0] - m[1][1]) * 2.f;
                X = (m[0][2] + m[0][2]) / S;
                Y = (m[1][2] + m[2][1]) / S;
                Z = 0.25f * S;
                W = (m[0][1] - m[1][0]) / S;
            }
        }

        coeff[0] = W; coeff[1] = -X; coeff[2] = -Y; coeff[3] = -Z;
    }


    /// Construct the quaternion from the a rotation axis 'axis' and the angle
    /// 'angle' in radians
    MyQuaternion(const Vec& axis, float angle)
    {
        Vec vec_axis = axis.unit();
        float sin_a = sin(angle * 0.5f);
        float cos_a = cos(angle * 0.5f);
        coeff[0] = cos_a;
        coeff[1] = vec_axis.x * sin_a;
        coeff[2] = vec_axis.y * sin_a;
        coeff[3] = vec_axis.z * sin_a;
        // It is necessary to normalize the quaternion in case any values are
        // very close to zero.
        normalize();
    }

    // -------------------------------------------------------------------------
    /// @name Methods
    // -------------------------------------------------------------------------

    /// The conjugate of a quaternion is the inverse rotation
    /// (when the quaternion is normalized
    MyQuaternion conjugate() const
    {
        return MyQuaternion(coeff[0], -coeff[1],
            -coeff[2], -coeff[3]);
    }

    // TODO: Construct the quaternion from the rotation axis 'vec' and the
    // angle 'angle'
    // Quat_cu(const Vec3& vec, float angle)

    /// Do the rotation of vector 'v' with the quaternion
    Vec rotate(const Vec& v) const
    {

        // The conventionnal way to rotate a vector
        /*
        Quat_cu tmp = *this;
        tmp.normalize();
        // Compute the quaternion inverse with
        Quat_cu inv = tmp.conjugate();
        // Compute q * v * inv; in order to rotate the vector v
        // to do so v must be expressed as the quaternion q(0, v.x, v.y, v.z)
        return (Vec3)(*this * Quat_cu(0, v) * inv);
        */

        // An optimized way to compute rotation
        MyQuaternion q ((*this * MyQuaternion(0, v) * conjugate()));
        return Vec(q.i(), q.j(), q.k());

    }




    Vec get_vec_part() const
    {
        return Vec(coeff[1], coeff[2], coeff[3]);
    }

    float norm() const
    {
        return sqrt(coeff[0] * coeff[0] +
            coeff[1] * coeff[1] +
            coeff[2] * coeff[2] +
            coeff[3] * coeff[3]);
    }

    float normalize()
    {
        float n = norm();
        coeff[0] /= n;
        coeff[1] /= n;
        coeff[2] /= n;
        coeff[3] /= n;
        return n;
    }

    float dot(const MyQuaternion& q) {
        return w() * q.w() + i() * q.i() + j() * q.j() + k() * q.k();
    }

    float w() const { return coeff[0]; }
    float i() const { return coeff[1]; }
    float j() const { return coeff[2]; }
    float k() const { return coeff[3]; }

    // -------------------------------------------------------------------------
    /// @name Operators
    // -------------------------------------------------------------------------

    MyQuaternion operator/ (float scalar) const
    {
        MyQuaternion q = *this;
        q.coeff[0] /= scalar;
        q.coeff[1] /= scalar;
        q.coeff[2] /= scalar;
        q.coeff[3] /= scalar;
        return q;
    }

    MyQuaternion operator/= (float scalar) {
        coeff[0] /= scalar;
        coeff[1] /= scalar;
        coeff[2] /= scalar;
        coeff[3] /= scalar;
        return *this;
    }

    MyQuaternion operator* (const MyQuaternion& q) const
    {
        return MyQuaternion(
            coeff[0] * q.coeff[0] - coeff[1] * q.coeff[1] - coeff[2] * q.coeff[2] - coeff[3] * q.coeff[3],
            coeff[0] * q.coeff[1] + coeff[1] * q.coeff[0] + coeff[2] * q.coeff[3] - coeff[3] * q.coeff[2],
            coeff[0] * q.coeff[2] + coeff[2] * q.coeff[0] + coeff[3] * q.coeff[1] - coeff[1] * q.coeff[3],
            coeff[0] * q.coeff[3] + coeff[3] * q.coeff[0] + coeff[1] * q.coeff[2] - coeff[2] * q.coeff[1]);
    }

    MyQuaternion operator* (float scalar) const
    {
        return MyQuaternion(coeff[0] * scalar,
            coeff[1] * scalar,
            coeff[2] * scalar,
            coeff[3] * scalar);
    }

    MyQuaternion operator+ (const MyQuaternion& q) const
    {
        return MyQuaternion(coeff[0] + q.coeff[0],
            coeff[1] + q.coeff[1],
            coeff[2] + q.coeff[2],
            coeff[3] + q.coeff[3]);
    }

    /// Get vector part
    operator Vec () const {
        return Vec(coeff[1], coeff[2], coeff[3]);
    }

    /// Get scalar part
    operator float() const {
        return coeff[0];
    }

    // -------------------------------------------------------------------------
    /// @name Attributes
    // -------------------------------------------------------------------------

    /// coeff[0], coeff[1], coeff[2], coeff[3] respectively
    /// w, i, j, k coefficients or W, X, Y, Z as noted in the F.A.Q
    float coeff[4];

};


