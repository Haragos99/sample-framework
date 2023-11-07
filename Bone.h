#pragma once

#include <QGLViewer/qglviewer.h>
#include "Matrix4.h"
#include <Eigen/Eigen>
using qglviewer::Vec;




// Probáljuk majd meg hogy csak a kiválasztot csont részen csináljuk a key framet.

struct Keyframe {
    Keyframe(float time, Vec& position,const Vec& angles)
        : time_(time), position_(position),angles_(angles) {}

    float time() const { return time_; }
    const Vec& position() const { return position_; }
    const Vec& angeles() const { return angles_; }
    Vec angles_;
    Vec position_;
    Vec selected_point;
private:
    float time_;
    
    
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

struct Tree {

    
    std::vector<Tree> child;
    Vec point;
    int id;
    bool choose = false;
    Vec original;
    Vec endframe;
    Vec position;
    Vec angel_;
    bool used = false;
    Mat4 mymatrix;
    std::vector<Keyframe> keyframes;
    qglviewer::Quaternion quaternion;
  
    Tree() {}
    Tree(Vec p, int i)
    {
        point = p;
        id = i;
        original = point;

    }
    void Quaternion_to_Matrix();

    void reset_quaternion(Tree& t);

    void reset_all(Tree& t);

    void Addframe(Tree& t, Keyframe& frame);

    void animatepoziton(Tree& t);
     
    void animaterotaion(Tree& t, float current_time);
   
    void change_all_position(Tree& t, Vec dif);

    void change_all_rotason(Tree& t, Vec pivot, Vec angles);
   
    void set_deafult_matrix(Tree& t);

    void used_points(Tree& t);

    Tree* searchbyid(Tree& t, int key);
   
    void drawarrow(Tree& t); 
    
    void makefalse(Tree& t);

    void maketrue(Tree& t);
  
    void drawchild(Tree& t);
    
};
 
struct Ecol {

};


