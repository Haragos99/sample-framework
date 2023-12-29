#include "MyViewer.h"
#include <math.h>

void MyViewer::inverse_kinematics(ControlPoint t, Tree& tree)
{
    ik.clear();
    tree_to_array(tree);
    double dis = abs(distance(ik[0], t.position));
    int start_index = tree.id;
    std::vector<double> distances;
    double total_distance = 0.0;
    //ik where we store the old points for the calculation of the ik
    for (int i = 0; i < ik.size() - 1; i++)
    {
        double d = abs(distance(ik[i], ik[i + 1]));
        distances.push_back(d);
        total_distance += d;

    }
    //ik = points;
    if(dis > total_distance)
    { 
        for (int i = 0; i < ik.size()-1; i++)
        {
            double r = abs(distance(t.position,ik[i]));
            double d = distances[i];
            double alfa = d / r;
            ik[i + 1] = (1 - alfa) * ik[i] + alfa * t.position;
        
        }
    }
    else 
    {  
        Vec b = ik[0];
        double diff = abs(distance(ik.back(), t.position));
        double tol = 0.001;
        int iter = 0;
        while (diff > tol && iter++ < 100)
        {
            ik.back() = t.position;
            for (int i = ik.size() - 2; i >= 0; i--)
            {
                double d = distances[i];
                double r = abs(distance(ik[i], ik[i + 1]));
                double alfa = d / r;
                ik[i] = (1 - alfa) * ik[i+1] + alfa * ik[i];

            }
            ik[0] = b;
            for (int i = 0; i < ik.size() - 2; i++)
            {
                double d = distances[i];
                double r = abs(distance(ik[i], ik[i + 1]));
                double alfa = d / r;
                ik[i+1] = (1 - alfa) * ik[i] + alfa * ik[i+1];
            }
            diff = abs(distance(ik.back(), t.position));
        }
    }
    // update and rebuild the full tree 
    Tree old_sk = sk;
    int j = 0;
    for (int i = start_index; i < points.size(); i++)
    {
        //points[i] = ik[j];
        Tree* s = tree.searchbyid(tree, i);
       
        s->point = ik[j];
        j++;

    }
    
    int bone_index = -1;
    FABRIK_p = ik;
    //IK_matrices();
    for (int i = 0; i < b.size(); i++)
    {
        for (int j = 0; j < points.size(); j++)
        {
            Tree* s = sk.searchbyid(sk, j);
            Tree* o = old_sk.searchbyid(old_sk, j);

            if (b[i].start == o->point)
            {
                b[i].start = s->point;
                bone_index = i;
            }
            if (b[i].End == o->point)
            {
                b[i].End = s->point;
                //  des = i;
            }
            if (bone_index != -1)
            {
                if (isweight)
                {
                    Tree* st = sk.searchbyid(sk, bone_index + 1);
                    b[bone_index].M = st->mymatrix;
                    bone_index = -1;
                }
            }
        }
    }

    animate_mesh();
    
    

    //ininitSkelton();
    //put_original(old_sk,sk);
}


void MyViewer::put_original(Tree& oldTree, Tree& newTree)
{
    newTree.original = oldTree.original;
    for (int i = 0; i < oldTree.child.size(); i++)
    {
        put_original(oldTree.child[i], newTree.child[i]);
    }
}

void MyViewer::IK_matrices()
{
    // TODOO: The piwot is wrong
    int n = ik.size();
    sk.reset_quaternion(sk);


    getallpoints(sk);
    std::vector<Vec> old_p = points;
    selected_points_storage.clear();

    sk.set_deafult_matrix(sk);
    qglviewer::Quaternion parentRotation = qglviewer::Quaternion();
    for (int i = 1; i < n; ++i)
    {
        Tree* t = sk.searchbyid(sk, i);


        Vec old_point = old_p[i] - old_p[i - 1];
        Vec new_point = ik[i] - ik[i - 1];
        old_point = old_point.unit();
        new_point = new_point.unit();
        Vec axis = old_point ^ new_point;
        float dot = old_point.x * new_point.x + old_point.y * new_point.y + old_point.z * new_point.z;
        float rotAngle = std::atan2((old_point ^ new_point).norm(), dot); //std::acos(dot / (mag1 * mag2));

        Vec pivot = old_p[i - 1];
        Mat4 T1 = TranslateMatrix(-pivot);
        Mat4 T2 = TranslateMatrix(pivot);

        Mat4 R;
        if (axis.norm() > 1E-12) {
            axis = axis.unit();
            R = RotationMatrix(rotAngle, axis);
            t->quaternion.setAxisAngle(axis, rotAngle);
        }
        Mat4 M = T1 * R * T2;
        Vec4 p = Vec4(old_p[i]) * M;
        t->mymatrix = M;
        t->point = Vec(p.x,p.y,p.z);

    }

    
    for (int i = 0; i < n; i++)
    {
        Tree* t = sk.searchbyid(sk, i);
        t->quaternion = qglviewer::Quaternion();
    }


}


void MyViewer::tree_to_array(Tree& t)
{
    
    Vec& r = t.point;
    ik.push_back(r);
    for (int i = 0; i < t.child.size(); i++)
    {
        tree_to_array(t.child[i]);
    }
}