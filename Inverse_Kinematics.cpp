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
        //Tree* s = tree.searchbyid(tree, i);
       
        //s->point = ik[j];
        j++;

    }
    
    int bone_index = -1;
    IK_matrices();
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
        }
    }
    


    //ininitSkelton();
   // put_original(old_sk,sk);
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
    Tree* tk = sk.searchbyid(sk, 1);
    Tree* to = sk.searchbyid(sk, 0);
    Vec old_point = tk->point- to->point;
    Vec new_point = ik[1] - ik[0];
    Vec axis = old_point ^ new_point;
    axis.unit();
    old_point.unit();
    new_point.unit();
    float mag1 = std::sqrt(old_point.x * old_point.x + old_point.y * old_point.y + old_point.z * old_point.z);
    float mag2 = std::sqrt(new_point.x * new_point.x + new_point.y * new_point.y + new_point.z * new_point.z);
    float dot = old_point.x * new_point.x + old_point.y * new_point.y + old_point.z * new_point.z;
    float rotAngle = std::acos(dot / (mag1 * mag2));

    qglviewer::Quaternion parentRotation = qglviewer::Quaternion();
    for (int i = 0; i < n; ++i)
    {
        Tree* t = sk.searchbyid(sk, i);
        
        t->quaternion.setAxisAngle(axis, rotAngle);
        t->point = t->quaternion.rotate(t->point);

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