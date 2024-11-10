#pragma once
#include <QGLViewer/qglviewer.h>
#include "Matrix4.h"
#include "Keyframe.h"
using qglviewer::Vec;






struct Joint {

    int id;// id of the join and the index of the position
    std::vector<int> bones_id;
    Vec point;
    Vec Tpose;
    Joint* parent;
    Vec angel;
    Vec pivot;
    std::vector<Joint*> children;
    std::vector<Keyframe> keyframes;
    Joint() { point = Vec(); }
    Joint(Vec p, int _id) { point = p; id = _id; Tpose = p; parent = nullptr; }
    Mat4 M;
    Mat4 R;

    void setMatrix(Vec& angel);

    Joint* searchbyid(Joint* j, int key);
    void drawarrow(Joint* j);

    void draw(Joint* j);
    void draw();

    void change_all_position(Joint* j, Vec dif);

    void change_all_rotason(Joint* j, Vec pivot, Vec angles);

    void set_deafult_matrix(Joint* j);

    void addframe(Joint* j, Keyframe& frame);

    void animaterotaion(Joint* j, float current_time, Mat4 M);

    void reset_all(Joint* j);

    void transform_point(Joint* j);

    Mat4 getMatrix();
    
    Joint* getLeaf(Joint* j);

    void calculateMatrecies(Joint* j, Vec pivot, Vec angles);


};



