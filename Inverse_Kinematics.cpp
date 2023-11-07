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
       
        //s->point = ik[j];
        j++;

    }
    
    int bone_index = -1;
    IK_matrices();
    


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
    qglviewer::Quaternion parentRotation = qglviewer::Quaternion();
    for (int i = 0; i < n-1; ++i)
    {

        if (i > 0)
        {
            Tree* parent = sk.searchbyid(sk, i - 1);
            parentRotation = parent->quaternion;
        }
        Vec baseDir = Vec(0, 1, 0);
        Vec newDir = parentRotation.inverse() * (ik[i + 1] - ik[i]);
        Vec rotAxis = normalize(normalize(baseDir) ^ normalize(newDir));
        rotAxis.x = 0;
        float mag1 = std::sqrt(baseDir.x * baseDir.x + baseDir.y * baseDir.y + baseDir.z * baseDir.z);
        float mag2 = std::sqrt(newDir.x * newDir.x + newDir.y * newDir.y + newDir.z * newDir.z);

        float dot = baseDir.x * newDir.x + baseDir.y * newDir.y + baseDir.z * newDir.z;
        
        float rotAngle = std::atan2((baseDir.unit() ^ newDir.unit()).norm(), dot);//std::acos(dot / (mag1 * mag2));
        if ((baseDir.unit() ^ newDir.unit()).norm() < 0.00001) { rotAngle = 0.0; }


        Tree* t = sk.searchbyid(sk, i );
        if (rotAngle < 0.001)
        {
            t->quaternion = qglviewer::Quaternion();
        }
        else
        {
            t->quaternion.setAxisAngle(rotAxis, rotAngle);
        }
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