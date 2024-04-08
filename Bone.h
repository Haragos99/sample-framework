#pragma once

#include "Matrix4.h"
#include <Eigen/Eigen>
#include "Mesh.h"
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







struct Bone {
    std::vector<int> joins_id;
    Joint* start;
    Joint* end;
    Mat4 M;
    int id;
    Vec color;
    std::vector<Vec> points;
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
        manypoints();

    }
    void draw();

    void manypoints();


};

struct Skelton {
private:
    std::vector<Vec> points;
    std::vector<Vec> Tpose;
    int n_joint;

public:
    Joint* root;
    std::vector<Bone> bones;
    std::vector<Vec> po;
    
    Skelton(std::vector<Vec> point) {
        points = point;
        Tpose = point;

    }
    Skelton() {  }

    int getSize() { return bones.size(); }

    void set_deafult_matrix() { root->set_deafult_matrix(root); }

    void get_join_point(Joint *j)
    {
            po.push_back(j->Tpose);
            for (int i = 0; i < j->children.size(); i++)
            {
                get_join_point(j->children[i]);
            }
        
    }
    std::vector<Vec> getPoints() { get_join_point(root); return po; }

    void animate(float current_time, MyMesh& mesh);

    void animate_mesh( MyMesh& mesh,bool isweight, bool inv = false);

    std::vector<Axes>arrows();

    void build(){
        root = new Joint(Vec(0,0,0),0);
        Joint* j = new Joint(Vec(0, 0.35, 0),1);
        Joint* j2 = new Joint(Vec(0, 0.7, 0), 2);
        Joint* j3 = new Joint(Vec(0, 1, 0),3);
        n_joint = 4;
        root->children.push_back(j);
        root->children[0]->children.push_back(j2);
        root->children[0]->children[0]->children.push_back(j3);

        bones.push_back(Bone(root, j,0,Vec(1,0,0)));
        bones.push_back(Bone(j, j2, 1, Vec(0, 1, 0)));
        bones.push_back(Bone(j2, j3,2, Vec(0, 0, 1)));
    }

    void drawarrow()
    {
        for (int i = 0; i < points.size(); i++)
        {
            Joint* j = root->searchbyid(root,i);
            Vec const& p = points[j->id];
            glPushName(j->id);
            glRasterPos3fv(p);
            glPopName();

        }
    }

    void draw()
    {
        for (auto b : bones)
        {
            b.draw();
        }
        root->draw(root);
    }

    void reset() { root->reset_all(root); }

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


/*
* TODO
* bone id -> joint id
* join id -> bone id
*/
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
 


