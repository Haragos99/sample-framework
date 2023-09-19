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
    qglviewer::Quaternion rotation_;
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
  
    Tree() {}
    Tree(Vec p, int i)
    {
        point = p;
        id = i;
        original = point;

    }

    void reset_all(Tree& t)
    {
        t.point = t.original;
        for (int i = 0; i < t.child.size(); i++)
        {
            reset_all(t.child[i]);
        }
    }

    void Addframe(Tree& t, Keyframe& frame)
    {
        frame.angles_ = frame.angles_ + t.point;
        t.keyframes.push_back(frame);
        
    }

    void animatepoziton(Tree& t)
    {
            endframe.x += 0.001f;
            Vec ir = 0.01f * endframe;
            change_all_position(t, ir);
            
        

    }
    // TODO: probáljuk  meg matrixokal.
    void animaterotaion(Tree& t,float current_time)
    {
         
        if (t.keyframes.size() != 0 && t.keyframes.back().time() > current_time)
        {
                size_t s = 0;
                // think again -2 
                while (s < t.keyframes.size() -2  && current_time >= t.keyframes[s + 1].time()) {
                    s++;
                }
                const Keyframe& startKeyframe = t.keyframes[s];
                const Keyframe& endKeyframe = t.keyframes[s + 1];
                float time = (current_time - startKeyframe.time()) / (endKeyframe.time() - startKeyframe.time());
                float timediff = (endKeyframe.time() - startKeyframe.time());
                Vec rotated = (endKeyframe.angeles() - startKeyframe.angeles());
                Vec angels = rotated / timediff;
                t.angel_ = angels;
                t.position = startKeyframe.position();
                change_all_rotason(t, startKeyframe.position(), angels); 

        }
        for (int j = 0; j < t.child.size(); j++)
        {
            animaterotaion(t.child[j], current_time);
        }
       
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="t"></param>
    /// <param name="newpos">az új hekye a kordinátának</param> 
    void change_all_position(Tree& t, Vec dif)
    {

        t.point += dif;
        // if (t.id == 0)return;
        for (int i = 0; i < t.child.size(); i++)
        {
            change_all_position(t.child[i], dif);
        }

    }




    void change_all_rotason(Tree& t, Vec orginal, Vec angles)
    {
        qglviewer::Quaternion qx = qglviewer::Quaternion(Vec(1, 0, 0), angles.x / 180.0 * M_PI);
        qglviewer::Quaternion qy = qglviewer::Quaternion(Vec(0, 1, 0), angles.y / 180.0 * M_PI);
        qglviewer::Quaternion qz = qglviewer::Quaternion(Vec(0, 0, 1), angles.z / 180.0 * M_PI);
        //t.angel = angles;
        qglviewer::Quaternion q = qz * qy * qx;
        double qmatrix[4][4];
        q.getMatrix(qmatrix);
        Mat4 R = transform_to_mat4(qmatrix);
        Mat4 T1 = TranslateMatrix(-orginal);
        Mat4 T2 = TranslateMatrix(orginal);
        Vec4 point4= Vec4(t.point.x, t.point.y, t.point.z, 1);

        if (t.point != orginal)
        {   
            t.angel_ += angles;
            //t.point = orginal + q.rotate(t.point - orginal);
            Mat4 M = T1 * R * T2;
            t.mymatrix = t.mymatrix * M;
            Vec4 result = point4 * M;
            t.point = Vec(result.x, result.y, result.z);
        }
        for (int i = 0; i < t.child.size(); i++)
        {
            change_all_rotason(t.child[i], orginal, angles);
        }
    }



    void set_deafult_matrix(Tree& t)
    {
        t.mymatrix = Mat4();
        for (int i = 0; i < t.child.size(); i++)
        {
            set_deafult_matrix(t.child[i]);
        }
    }


    void used_points(Tree& t)
    {
        t.used = true;
        for (int i = 0; i < t.child.size(); i++)
        {
            used_points(t.child[i]);
        }
    }

    Tree* searchbyid(Tree& t, int key)
    {
        // std::cout << t.id << "\n";
        //t.choose = true;
        if (key == t.id) { return &t; }
        
        for (int i = 0; i < t.child.size(); i++)
        {
            Tree* result = searchbyid(t.child[i], key);
            if (result != nullptr)
                return result;
        }
        return nullptr;
    }
    void drawarrow(Tree& t)
    {
        Vec const& p = t.point;
        glPushName(t.id);
        glRasterPos3fv(p);
        glPopName();
        for (int i = 0; i < t.child.size(); i++)
        {
            drawarrow(t.child[i]);
        }
    }

    void makefalse(Tree& t)
    {
        t.choose = false;
        for (int i = 0; i < t.child.size(); i++)
        {
            makefalse(t.child[i]);
        }
    }

    void maketrue(Tree& t)
    {
        t.choose = true;
        for (int i = 0; i < t.child.size(); i++)
        {
            maketrue(t.child[i]);
        }
    }



    void drawchild(Tree& t)
    {
        // if (t.child.size() == 0) return;
        Vec const& p = t.point;
        glDisable(GL_LIGHTING);
        if(t.choose)
            glColor3d(1.0, 0.0, 1.0);
        else
            glColor3d(0.0, 0.0, 1.0);
        glPointSize(50.0);
        glBegin(GL_POINTS);
        glVertex3dv(p);
        glEnd();
        glPointSize(10.0);
        glEnable(GL_LIGHTING);
        for (int i = 0; i < t.child.size(); i++)
        {
            drawchild(t.child[i]);
        }

    }
};
