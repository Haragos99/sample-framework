#pragma once

#include <QGLViewer/qglviewer.h>
using qglviewer::Vec;



struct Keyframe {
    Keyframe(float time, Vec& position,const Vec& angles)
        : time_(time), position_(position),angles_(angles) {}

    float time() const { return time_; }
    const Vec& position() const { return position_; }
    const Vec& angeles() const { return angles_; }
    Vec angles_;
    Vec position_;
private:
    float time_;
    
    
};



struct Bones
{
    Vec start;
    Vec End;
    
    Vec originalS; 
    Vec originalE;

    std::vector<Vec> points;
    double x, y, z;

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
    Vec angel;
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

    void Addframe(Tree& t, Keyframe frame)
    {
        frame.angles_ = frame.angles_ + t.point;
        t.keyframes.push_back(frame);
        for (int i = 0; i < t.child.size(); i++)
        {
            Addframe(t.child[i], frame);
        }
    }

    void animatepoziton(Tree& t)
    {
            endframe.x += 0.001f;
            Vec ir = 0.01f * endframe;
            change_all_position(t, ir);
            
        

    }

    void animaterotaion(Tree& t, int i,float current_time,Vec original)
    {
         
        if (t.keyframes.size()!=0)
        {
            const Keyframe& startKeyframe = t.keyframes[i];
            const Keyframe& endKeyframe = t.keyframes[i + 1];
            float time = (current_time - startKeyframe.time()) / (endKeyframe.time() - startKeyframe.time());
            Vec angels = startKeyframe.angeles() + 1 * (endKeyframe.angeles() - startKeyframe.angeles());

            //change_all_rotason(t, original, angels);
            qglviewer::Quaternion qx = qglviewer::Quaternion(Vec(1, 0, 0), angels.x / 180.0 * M_PI);
            qglviewer::Quaternion qy = qglviewer::Quaternion(Vec(0, 1, 0), angels.y / 180.0 * M_PI);
            qglviewer::Quaternion qz = qglviewer::Quaternion(Vec(0, 0, 1), angels.z / 180.0 * M_PI);
            //t.angel = angles;
            if (t.point != original)
            {
                t.point = original + qx.rotate(t.point - original);

                t.point = original + qy.rotate(t.point - original);


                t.point = original + qz.rotate(t.point - original);

            }


        }
        for (int j = 0; j < t.child.size(); j++)
        {
            animaterotaion(t.child[j], i, current_time,original);
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
        if (t.point != orginal)
        {
            t.point = orginal + qx.rotate(t.point - orginal);

            t.point = orginal + qy.rotate(t.point - orginal);


            t.point = orginal + qz.rotate(t.point - orginal);

        }
        for (int i = 0; i < t.child.size(); i++)
        {
            change_all_rotason(t.child[i], orginal, angles);
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







