
#include "Joint.h"

void Joint::change_all_position(Joint* j, Vec dif)
{
    j->point += dif;
    for (int i = 0; i < j->children.size(); i++)
    {
        change_all_position(j->children[i], dif);
    }
}




void Joint::change_all_rotason(Joint* j, Vec pivot, Vec angles)
{
    qglviewer::Quaternion qx = qglviewer::Quaternion(Vec(1, 0, 0), angles.x / 180.0 * M_PI);
    qglviewer::Quaternion qy = qglviewer::Quaternion(Vec(0, 1, 0), angles.y / 180.0 * M_PI);
    qglviewer::Quaternion qz = qglviewer::Quaternion(Vec(0, 0, 1), angles.z / 180.0 * M_PI);
    qglviewer::Quaternion q = qz * qy * qx;
    double qmatrix[4][4];
    q.getMatrix(qmatrix);
    Mat4 R = transform_to_mat4(qmatrix);
    j->R = R;
    Mat4 T1 = TranslateMatrix(-pivot);
    Mat4 T2 = TranslateMatrix(pivot);
    Vec4 point4 = Vec4(j->point.x, j->point.y, j->point.z, 1);

    if (j->point != pivot)
    {
        Mat4 M = T1 * R * T2;
        j->M = j->M * M;
        Vec4 result = point4 * j->M;
        j->point = Vec(result.x, result.y, result.z);

    }
    for (int i = 0; i < j->children.size(); i++)
    {
        change_all_rotason(j->children[i], pivot, angles);
    }
}


void Joint::setMatrix(Vec& angel)
{
    R = RotationMatrix(angel.z / 180.0 * M_PI, Vec(0, 0, 1)) * RotationMatrix(angel.y / 180.0 * M_PI, Vec(0, 1, 0))* RotationMatrix(angel.x / 180.0 * M_PI, Vec(1, 0, 0));
    M = R;  
}


void Joint::draw(Joint* j)
{
    Vec const& p = j->point;
    glDisable(GL_LIGHTING);

    glColor3d(1.0, 0.0, 1.0);
    glPointSize(50.0);
    glBegin(GL_POINTS);
    glVertex3dv(p);
    glEnd();
    glEnable(GL_LIGHTING);
    for (int i = 0; i < j->children.size(); i++)
    {
        draw(j->children[i]);
    }
}


Joint* Joint::searchbyid(Joint* j, int key)
{
    if (key == j->id) { return j; }

    for (int i = 0; i < j->children.size(); i++)
    {
        Joint* result = searchbyid(j->children[i], key);
        if (result != nullptr)
            return result;
    }
    return nullptr;
}


Joint* Joint::getLeaf(Joint* j)
{
    if ( j->children.size()==0) { return j; }

    for (int i = 0; i < j->children.size(); i++)
    {
        Joint* result = getLeaf(j->children[i]);
        if (result != nullptr)
            return result;
    }
}



void Joint::drawarrow(Joint* j)
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

void Joint::set_deafult_matrix(Joint* j)
{
    j->M = Mat4();
    for (int i = 0; i < j->children.size(); i++)
    {
        set_deafult_matrix(j->children[i]);
    }
}


void Joint::addframe(Joint* j, Keyframe& frame)
{
    j->keyframes.push_back(frame);
}


void Joint::animaterotaion(Joint* j, float current_time, Mat4 M)
{
    Vec4 point4 = Vec4(j->point.x, j->point.y, j->point.z, 1);
    j->M = j->M * M;
    Vec4 result = point4 * j->M;
    j->point = Vec(result.x, result.y, result.z);
    if (j->keyframes.size() != 0 && j->keyframes.back().time() >= current_time)
    {
        size_t s = 0;

        // think again -2 
        while (s < j->keyframes.size() - 2 && current_time >= j->keyframes[s + 1].time()) {
            s++;
        }
        const Keyframe& startKeyframe = j->keyframes[s];
        const Keyframe& endKeyframe = j->keyframes[s + 1];
        float timediff_key = (endKeyframe.time() - startKeyframe.time());
        float dt = current_time - startKeyframe.time();
        Vec rotated = (endKeyframe.angeles() - startKeyframe.angeles());
        float r = (dt / timediff_key);
        Vec angels = rotated * r / 10;
        if (dt >= endKeyframe.time())angels = Vec(0, 0, 0);
        Vec pivot = j->point;
        /*
        Mat4 T1 = TranslateMatrix(-pivot);
        Mat4 T2 = TranslateMatrix(pivot);
        qglviewer::Quaternion qx = qglviewer::Quaternion(Vec(1, 0, 0), angels.x / 180.0 * M_PI);
        qglviewer::Quaternion qy = qglviewer::Quaternion(Vec(0, 1, 0), angels.y / 180.0 * M_PI);
        qglviewer::Quaternion qz = qglviewer::Quaternion(Vec(0, 0, 1), angels.z / 180.0 * M_PI);
        qglviewer::Quaternion q = qz * qy * qx;
        double qmatrix[4][4];
        q.getMatrix(qmatrix);
        Mat4 R = transform_to_mat4(qmatrix);


        Mat4 _M = T1 * R * T2;
        M = M * _M;
        */
        change_all_rotason(j, pivot, angels);


    }
    for (int i = 0; i < j->children.size(); i++)
    {
        animaterotaion(j->children[i], current_time, M);
    }

}


void Joint::reset_all(Joint* j)
{
    j->point = j->Tpose;
    j->M = Mat4();
    j->R = Mat4();
    for (int i = 0; i < j->children.size(); i++)
    {
        reset_all(j->children[i]);
    }
}



Mat4 Joint::getMatrix()
{
    Mat4 R = RotationMatrix(angel.z, pivot) * RotationMatrix(angel.y, pivot) * RotationMatrix(angel.x, pivot);
    Mat4 T1 = TranslateMatrix(-pivot);
    Mat4 T2 = TranslateMatrix(pivot);

    return T1 * R * T2;
}



void Joint::calculateMatrecies(Joint* j, Vec _pivot, Vec angles)
{
    qglviewer::Quaternion qx = qglviewer::Quaternion(Vec(1, 0, 0), angles.x / 180.0 * M_PI);
    qglviewer::Quaternion qy = qglviewer::Quaternion(Vec(0, 1, 0), angles.y / 180.0 * M_PI);
    qglviewer::Quaternion qz = qglviewer::Quaternion(Vec(0, 0, 1), angles.z / 180.0 * M_PI);
    qglviewer::Quaternion q = qz * qy * qx;
    double qmatrix[4][4];
    q.getMatrix(qmatrix);
    Mat4 R = transform_to_mat4(qmatrix);
    j->R = R;
    Mat4 T1 = TranslateMatrix(-_pivot);
    Mat4 T2 = TranslateMatrix(_pivot);
    if (j->point != _pivot)
    {
        Mat4 M = T1 * R * T2;
        j->M = j->M * M;
    }
    for (int i = 0; i < j->children.size(); i++)
    {
        calculateMatrecies(j->children[i], _pivot, angles);
    }
}

void Joint::transform_point(Joint* j)
{
    Vec4 point4 = Vec4(j->point.x, j->point.y, j->point.z, 1);
    Vec4 result = point4 * j->M;
    j->point = Vec(result.x, result.y, result.z);
    for (int i = 0; i < j->children.size(); i++)
    {
        transform_point(j->children[i]);
    }
}