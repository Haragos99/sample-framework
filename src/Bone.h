#pragma once

#include "Matrix4.h"
#include <fstream>
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

class Skelton {
private:
    std::vector<Vec> points;
    std::vector<Vec> Tpose;
    int n_joint;
    std::vector< std::pair<int, int>> indexes;
    std::vector<std::vector<int>> childrenMatrix;
    std::vector<Vec> colors_bone{
    Vec(0.0, 1.0, 1.0),
    Vec(1.0, 1.0, 0.0),
    Vec(1.0, 0.0, 1.0),
    Vec(0.5, 1.0, 0.5),
    Vec(1.0, 0.5, 0.5),
    Vec(0.5, 0.5, 1.0),
    Vec(0.1, 0.2, 0.2),
    Vec(0.7, 0.3, 0.0),
    Vec(0.0, 0.3, 0.7),
    Vec(0.0, 0.7, 0.3),
    Vec(0.7, 0.0, 0.3),
    Vec(0.3, 0.0, 0.7),
    Vec(0.3, 0.7, 0.0),
    Vec(0.7, 0.0, 0.0),
    Vec(0.0, 0.7 ,0.0),
    Vec(0.0, 0.0, 0.7),
    Vec(0.7, 0.7, 0.7),
    Vec(0.5, 1.0, 0.2),
    Vec(1.0, 0.6, 0.2),
    Vec(0.4, 0.5, 1.0),
    Vec(0.1, 0.2, 0.2),
    Vec(0.5, 0.3, 0.0),
    Vec(0.1, 0.3, 0.7),
    Vec(0.1, 0.7, 0.3),
    };


public:
    Joint* root;
    std::vector<Bone> bones;
    std::vector<Vec> po;
    std::vector<Joint*> joint;
    
    Skelton(std::vector<Vec> point, std::vector<std::vector<int>> _childrenMatrix, std::vector< std::pair<int, int>> _indexes) {
        points = point;
        Tpose = point;
        childrenMatrix = _childrenMatrix;
        indexes = _indexes;
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

    bool hasMultipleChildren(Joint* j) {
        getList(j);
        for (auto j : joint)
        {
            if (j->children.size() > 1)
            {
                return false;
            }
        }
        return true;
    }

    void loadFile(const std::string& filename);

    void getList(Joint* j)
    {
        joint.push_back(j);
        for (int i = 0; i < j->children.size(); i++)
        {
            getList(j->children[i]);
        }
    }

    void setJointMatrix(int id, Vec& angle);

    std::vector<Vec> getPoints(Joint* j) { get_join_point(j); return po; }

    std::vector<Joint*> getJointtoList(Joint* j) { getList(j); return joint; }

    void animate(float current_time, MyMesh& mesh);

    void animate_mesh( MyMesh& mesh,bool isweight, bool inv = false);

    std::vector<Axes>arrows();

    void build();

    std::vector<Vec> getPointlist() { return points; }

    void buildTree(std::vector<Joint*>& joints);

    void addJoint(Joint* parent, Joint* child);

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

    bool save(const std::string& filename);

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



 


