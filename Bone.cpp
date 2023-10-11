#include "Bone.h"



void Tree::reset_all(Tree& t)
{
    t.point = t.original;
    t.angel_ = Vec(0, 0, 0);
    for (int i = 0; i < t.child.size(); i++)
    {
        reset_all(t.child[i]);
    }
}

void Tree::Addframe(Tree& t, Keyframe& frame)
{
    frame.angles_ = frame.angles_ + t.point;
    t.keyframes.push_back(frame);

}

void Tree::animatepoziton(Tree& t)
{
    endframe.x += 0.001f;
    Vec ir = 0.01f * endframe;
    change_all_position(t, ir);



}
void Tree::animaterotaion(Tree& t, float current_time)
{

    if (t.keyframes.size() != 0 && t.keyframes.back().time() > current_time)
    {
        size_t s = 0;
        // think again -2 
        while (s < t.keyframes.size() - 2 && current_time >= t.keyframes[s + 1].time()) {
            s++;
        }
        const Keyframe& startKeyframe = t.keyframes[s];
        const Keyframe& endKeyframe = t.keyframes[s + 1];
        float timediff_key = (endKeyframe.time() - startKeyframe.time());
        float dt = current_time - startKeyframe.time();
        Vec rotated = (endKeyframe.angeles() - startKeyframe.angeles());
        Vec angels = rotated * (dt / timediff_key);
        t.angel_ = angels;
        t.position = startKeyframe.position();
        change_all_rotason(t, startKeyframe.position(), angels);

    }
    for (int j = 0; j < t.child.size(); j++)
    {
        animaterotaion(t.child[j], current_time);
    }

}

void Tree::change_all_position(Tree& t, Vec dif)
{
    t.point += dif;
    for (int i = 0; i < t.child.size(); i++)
    {
        change_all_position(t.child[i], dif);
    }

}


void Tree::change_all_rotason(Tree& t, Vec pivot, Vec angles)
{
    qglviewer::Quaternion qx = qglviewer::Quaternion(Vec(1, 0, 0), angles.x / 180.0 * M_PI);
    qglviewer::Quaternion qy = qglviewer::Quaternion(Vec(0, 1, 0), angles.y / 180.0 * M_PI);
    qglviewer::Quaternion qz = qglviewer::Quaternion(Vec(0, 0, 1), angles.z / 180.0 * M_PI);
    //t.angel = angles;
    qglviewer::Quaternion q = qz * qy * qx;
    double qmatrix[4][4];
    q.getMatrix(qmatrix);
    Mat4 R = transform_to_mat4(qmatrix);
    Mat4 T1 = TranslateMatrix(-pivot);
    Mat4 T2 = TranslateMatrix(pivot);
    Vec4 point4 = Vec4(t.original.x, t.original.y, t.original.z, 1);

    if (t.point != pivot)
    {
        t.angel_ += angles;
        Mat4 M = T1 * R * T2;
        t.mymatrix = M;//t.mymatrix * M;
        Vec4 result = point4 * t.mymatrix;
        t.point = Vec(result.x, result.y, result.z);

    }
    for (int i = 0; i < t.child.size(); i++)
    {
        change_all_rotason(t.child[i], pivot, angles);
    }
}

void Tree::set_deafult_matrix(Tree& t)
{
    t.mymatrix = Mat4();
    for (int i = 0; i < t.child.size(); i++)
    {
        set_deafult_matrix(t.child[i]);
    }
}


void Tree::used_points(Tree& t)
{
    t.used = true;
    for (int i = 0; i < t.child.size(); i++)
    {
        used_points(t.child[i]);
    }
}

Tree* Tree::searchbyid(Tree& t, int key)
{
    if (key == t.id) { return &t; }

    for (int i = 0; i < t.child.size(); i++)
    {
        Tree* result = searchbyid(t.child[i], key);
        if (result != nullptr)
            return result;
    }
    return nullptr;
}
void Tree::drawarrow(Tree& t)
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

void Tree::makefalse(Tree& t)
{
    t.choose = false;
    for (int i = 0; i < t.child.size(); i++)
    {
        makefalse(t.child[i]);
    }
}

void Tree::maketrue(Tree& t)
{
    t.choose = true;
    for (int i = 0; i < t.child.size(); i++)
    {
        maketrue(t.child[i]);
    }
}



void Tree::drawchild(Tree& t)
{
    Vec const& p = t.point;
    glDisable(GL_LIGHTING);
    if (t.choose)
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