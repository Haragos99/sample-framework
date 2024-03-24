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







struct Join {
    int id;// id of the join and the index of the position
    std::vector<int> bones_id;
    Vec point;
    Vec Tpose;
    Join* parent;
    std::vector<Join*> children;
    Join(){}
    Join(Vec p, int _id) { point = p; id = _id; Tpose = p; }
    Mat4 M;
    Join* searchbyid(Join* j, int key)
    {
        if (key == j->id) { return j; }

        for (int i = 0; i < j->children.size(); i++)
        {
            Join* result = searchbyid(j->children[i], key);
            if (result != nullptr)
                return result;
        }
        return nullptr;
    }
    void drawarrow(Join* j)
    {
        Vec const& p = j->point;
        glPushName(j->id);
        glRasterPos3fv(p);
        glPopName();
        for (int i = 0; i < j->children.size(); i++)
        {
            drawarrow(j->children[i]);
        }
    }
    void draw(Join* j)
    {
        Vec const& p = j->point;
        glDisable(GL_LIGHTING);
        
        glColor3d(1.0, 0.0, 1.0);
        glPointSize(50.0);
        glBegin(GL_POINTS);
        glVertex3dv(p);
        glEnd();
        glPointSize(10.0);
        glEnable(GL_LIGHTING);
        for (int i = 0; i < j->children.size(); i++)
        {
            draw(j->children[i]);
        }
    }
    void change_all_position(Join* j, Vec dif);

    void change_all_rotason(Join* j, Vec pivot, Vec angles);

    void set_deafult_matrix(Join* j)
    {
        j->M= Mat4();
        for (int i = 0; i < j->children.size(); i++)
        {
            set_deafult_matrix(j->children[i]);
        }
    }

};

struct Bone {
    std::vector<int> joins_id;
    Join* start;
    Join* end;
    Mat4 M;
    int id;
    Vec color;
    std::vector<Vec> points;
    Bone(Join* s, Join* e,int _id,Vec _color)
    {
        start = s;
        end = e;
        id = _id;
        joins_id.push_back(s->id);
        joins_id.push_back(e->id);
        color = _color;
        manypoints();

    }
    void draw();

    void manypoints()
    {

        Vec ir = end->point - start->point ;
        double len = sqrt(pow(end->point.x - start->point.x, 2) + 
            pow(end->point.y - start->point.y, 2) + pow(end->point.z - start->point.z, 2));
        int res = 100;
        for (int i = 0; i < res; i++)
        {
            Vec t;
            t = start->point + ir * 1.0 / (res - 1) * i;
            points.push_back(t);
        }
    }


};

struct Skelton {
private:
    std::vector<Vec> points;
    std::vector<Vec> Tpose;

public:
    Join* root;
    std::vector<Bone> bones;
    std::vector<Vec> po;
    
    Skelton(std::vector<Vec> point) {
        points = point;
        Tpose = point;

    }
    Skelton() {  }

    int getSize() { return bones.size(); }
    void Rotate(int id)
    {
         
    }
    void set_deafult_matrix() { root->set_deafult_matrix(root); }

    void get_join_point(Join *j)
    {
            po.push_back(j->Tpose);
            for (int i = 0; i < j->children.size(); i++)
            {
                get_join_point(j->children[i]);
            }
        
    }
    std::vector<Vec> getPoints() { get_join_point(root); return po; }

    void build(){
        root = new Join(Vec(0,0,0),0);
        Join* j = new Join(Vec(0, 0.35, 0),1);
        Join* j2 = new Join(Vec(0, 0.65, 0), 2);
        Join* j3 = new Join(Vec(0, 1, 0),3);
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
            Join* j = root->searchbyid(root,i);
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
 


