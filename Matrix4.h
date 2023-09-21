#pragma once
#include <iostream>
#include <iomanip>
#include <cmath>
#include <QGLViewer/qglviewer.h>
using qglviewer::Vec;

struct Matrix4 {
private:
    double data[4][4];

public:
    // Constructor
    Matrix4() {
        // Initialize the matrix as an identity matrix
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                data[i][j] = (i == j) ? 1.0 : 0.0;
            }
        }
    }

    // Getter and Setter for individual elements
    double get(int row, int col) const {
        if (row >= 0 && row < 4 && col >= 0 && col < 4) {
            return data[row][col];
        }
        else {
            std::cerr << "Invalid row or column index!" << std::endl;
            return 0.0; // Return 0 if indices are out of bounds
        }
    }

    void set(int row, int col, double value) {
        if (row >= 0 && row < 4 && col >= 0 && col < 4) {
            data[row][col] = value;
        }
        else {
            std::cerr << "Invalid row or column index!" << std::endl;
        }
    }
    // Setter using a 4x4 double array
    void set(const double matrix[4][4]) {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                data[i][j] = matrix[i][j];
            }
        }
    }


    void set_transfrom(const Vec v)
    {
        data[3][0] = v.x;
        data[3][1] = v.y;
        data[3][2] = v.z;
    }
    // Multiplication operator overload
    Matrix4 operator*(const Matrix4& other) const {
        Matrix4 result;

        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                result.data[i][j] = 0;
                for (int k = 0; k < 4; ++k) {
                    result.data[i][j] += data[i][k] * other.data[k][j];
                }
            }
        }

        return result;
    }

    

    // Multiplication operator overload for matrix-vector multiplication
    Vec operator*(const Vec& v) const {
        double result_x = data[0][0] * v.x + data[0][1] * v.y + data[0][2] * v.z + data[0][3] * 1;
        double result_y = data[1][0] * v.x + data[1][1] * v.y + data[1][2] * v.z + data[1][3] * 1;
        double result_z = data[2][0] * v.x + data[2][1] * v.y + data[2][2] * v.z + data[2][3] * 1;
        double result_w = data[3][0] * v.x + data[3][1] * v.y + data[3][2] * v.z + data[3][3] * 1;

        return Vec(result_x/ result_w, result_y/ result_w, result_z/ result_w);
    }

   

};
/*
inline Matrix4 RotationMatrix(double angle, Vec w)
{
    Matrix4 result;
    double c = cosf(angle), s = sinf(angle);
    double mat4[4][4] = {
        {c * (1 - w.x * w.x) + w.x * w.x, w.x * w.y * (1 - c) + w.z * s, w.x * w.z * (1 - c) - w.y * s, 0},
        {w.x * w.y * (1 - c) - w.z * s, c * (1 - w.y * w.y) + w.y * w.y, w.y * w.z * (1 - c) + w.x * s, 0},
        {w.x * w.z * (1 - c) + w.y * s, w.y * w.z * (1 - c) - w.x * s, c * (1 - w.z * w.z) + w.z * w.z, 0},
        {0, 0, 0, 1}
    };
    result.set(mat4);
    return result;
}

*/









inline float dot(const Vec& v1, const Vec& v2) { return (v1.x * v2.x + v1.y * v2.y + v1.z * v2.z); }

inline float length(const Vec& v) { return sqrtf(dot(v, v)); }

inline Vec normalize(const Vec& v) { return v * (1 / length(v)); }

struct Vec4 {
    //--------------------------
    float x, y, z, w;

    Vec4(float x0 = 0, float y0 = 0, float z0 = 0, float w0 = 0) { x = x0; y = y0; z = z0; w = w0; }
    float& operator[](int j) { return *(&x + j); }
    float operator[](int j) const { return *(&x + j); }

    Vec4 operator*(float a) const { return Vec4(x * a, y * a, z * a, w * a); }
    Vec4 operator/(float d) const { return Vec4(x / d, y / d, z / d, w / d); }
    Vec4 operator+(const Vec4& v) const { return Vec4(x + v.x, y + v.y, z + v.z, w + v.w); }
    Vec4 operator-(const Vec4& v)  const { return Vec4(x - v.x, y - v.y, z - v.z, w - v.w); }
    Vec4 operator*(const Vec4& v) const { return Vec4(x * v.x, y * v.y, z * v.z, w * v.w); }
    void operator+=(const Vec4 right) { x += right.x; y += right.y; z += right.z; w += right.w; }
};

inline float dot(const Vec4& v1, const Vec4& v2) {
    return (v1.x * v2.x + v1.y * v2.y + v1.z * v2.z + v1.w * v2.w);
}

inline Vec4 operator*(float a, const Vec4& v) {
    return Vec4(v.x * a, v.y * a, v.z * a, v.w * a);
}

//---------------------------
struct Mat4 { // row-major matrix 4x4
//---------------------------
    Vec4 rows[4];
public:
    Mat4() {
        rows[0] = Vec4(1, 0, 0, 0);
        rows[1] = Vec4(0, 1, 0, 0);
        rows[2] = Vec4(0, 0, 1, 0);
        rows[3] = Vec4(0, 0, 0, 1);
    }
    Mat4(float m00, float m01, float m02, float m03,
        float m10, float m11, float m12, float m13,
        float m20, float m21, float m22, float m23,
        float m30, float m31, float m32, float m33) {
        rows[0][0] = m00; rows[0][1] = m01; rows[0][2] = m02; rows[0][3] = m03;
        rows[1][0] = m10; rows[1][1] = m11; rows[1][2] = m12; rows[1][3] = m13;
        rows[2][0] = m20; rows[2][1] = m21; rows[2][2] = m22; rows[2][3] = m23;
        rows[3][0] = m30; rows[3][1] = m31; rows[3][2] = m32; rows[3][3] = m33;
    }
    Mat4(Vec4 it, Vec4 jt, Vec4 kt, Vec4 ot) {
        rows[0] = it; rows[1] = jt; rows[2] = kt; rows[3] = ot;
    }
    Vec4& operator[](int i) { return rows[i]; }
    Vec4 operator[](int i) const { return rows[i]; }
    operator float* () const { return (float*)this; }
    Mat4 skalar(double s)
    {
        Vec4 it = Vec4(rows[0].x * s, rows[0].y * s, rows[0].z * s, rows[0].w * s);
        Vec4 jt = Vec4(rows[1].x * s, rows[1].y * s, rows[1].z * s, rows[1].w * s);
        Vec4 kt = Vec4(rows[2].x * s, rows[2].y * s, rows[2].z * s, rows[2].w * s);
        Vec4 ot = Vec4(rows[3].x * s, rows[3].y * s, rows[3].z * s, rows[3].w * s);
        return Mat4(it, jt, kt, ot);
    }
    Mat4& operator+=(const Mat4& other) {
        for (int i = 0; i < 4; ++i) {
            rows[i] += other.rows[i];
        }
        return *this;
    }
};

inline Vec4 operator*(const Vec4& v, const Mat4& mat) {
    return v[0] * mat[0] + v[1] * mat[1] + v[2] * mat[2] + v[3] * mat[3];
}

inline Mat4 operator*(const Mat4& left, const Mat4& right) {
    Mat4 result;
    for (int i = 0; i < 4; i++) result.rows[i] = left.rows[i] * right;
    return result;
}

inline Mat4 TranslateMatrix(Vec t) {
    return Mat4(Vec4(1, 0, 0, 0),
        Vec4(0, 1, 0, 0),
        Vec4(0, 0, 1, 0),
        Vec4(t.x, t.y, t.z, 1));
}


inline Mat4 transform_to_mat4(double M[4][4])
{
    return Mat4(M[0][0], M[0][1], M[0][2], M[0][3],
                M[1][0], M[1][1], M[1][2], M[1][3],
                M[2][0], M[2][1], M[2][2], M[2][3],
                M[3][0], M[3][1], M[3][2], M[3][3]);
}

inline Mat4 ScaleMatrix(Vec s) {
    return Mat4(Vec4(s.x, 0, 0, 0),
        Vec4(0, s.y, 0, 0),
        Vec4(0, 0, s.z, 0),
        Vec4(0, 0, 0, 1));
}

inline Mat4 RotationMatrix(float angle, Vec w) {
    float c = cosf(angle), s = sinf(angle);
    w = normalize(w);
    return Mat4(Vec4(c * (1 - w.x * w.x) + w.x * w.x, w.x * w.y * (1 - c) + w.z * s, w.x * w.z * (1 - c) - w.y * s, 0),
        Vec4(w.x * w.y * (1 - c) - w.z * s, c * (1 - w.y * w.y) + w.y * w.y, w.y * w.z * (1 - c) + w.x * s, 0),
        Vec4(w.x * w.z * (1 - c) + w.y * s, w.y * w.z * (1 - c) - w.x * s, c * (1 - w.z * w.z) + w.z * w.z, 0),
        Vec4(0, 0, 0, 1));
}



