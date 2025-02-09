#pragma once

#include "../Matrix4.h"
#include <fstream>
#include <Eigen/Eigen>
#include "../Mesh.h"
#include "Joint.h"

using qglviewer::Vec;
using MyMesh = OpenMesh::TriMesh_ArrayKernelT<MyTraits>;
using Vector = OpenMesh::VectorT<double, 3>;



// Probáljuk majd meg hogy csak a kiválasztot csont részen csináljuk a key framet.


struct Axes {
    bool shown;
    float size;
    int selected_axis;
    Vec position, grabbed_pos, original_pos;
    Mat4 M;
    Axes(Vec _position, float _size, Mat4 _M) { position = _position; size = _size; M = _M; }
};




struct BonePoly {
    std::vector<Vec> points;
    Vec color;
    Joint* start;
    Joint* end;
    MyMesh mesh;
    Vec top, down, aP, bP, cP, dP; // Point of the rombus
    BonePoly(){}
    BonePoly(Joint* _start, Joint* _end, Vec& _color) { start = _start; end = _end; color = _color; calculatepoly(); };
    void calculatepoly();
    Vec calcnormal(Vec& ab, Vec& ac) { return (ab ^ ac).unit(); }
    void draw();
    void drawface(Vec& a, Vec& b, Vec& c);
    void drawLines(Vec& a, Vec& b);
};




struct Bone {
    std::vector<int> joins_id;
    Joint* start;
    Joint* end;
    Mat4 M;
    int id;
    Vec color;
    BonePoly bp;
    std::vector<Vec> points;
    Bone():start(nullptr), end(nullptr) {}
    Bone(Joint* s, Joint* e,int _id,Vec _color)
    {
        start = s;
        end = e;
        id = _id;
        joins_id.push_back(s->id);
        joins_id.push_back(e->id);
        color = _color;
        end->parent = s;
        start->bones_id.push_back(id);
        end->bones_id.push_back(id);
        bp = BonePoly(start, end,color);
        manypoints();


    }
    void draw();

    void manypoints();

    bool isLastBone();

    float lenght() { return (start->point - end->point).norm(); }


};




struct Bones
{
    Vec start;
    Vec End;
    bool last_bone = false;
    Vec originalS; 
    Vec originalE;
    std::vector<Keyframe> keyframes;
    std::vector<Vec> points;
    double lenght;
    Mat4 M;
    double x, y, z;
    // ide egy matrix írjunk
    Vec getColor()
    {
        return Vec(x, y, z);
    }


    void setColor(double _x, double _y, double _z)
    {
        x = _x;
        y = _y;
        z = _z;
    }

    void manypoints()
    {

        Vec ir = End - start;
        double len = sqrt(pow(End.x - start.x, 2) + pow(End.y - start.y, 2) + pow(End.z - start.z, 2));
        int res = 100;
        for (int i = 0; i < res; i++)
        {
            Vec t;
            t = start + ir * 1.0 / (res-1) * i;
            points.push_back(t);
        }
    }
};



 


