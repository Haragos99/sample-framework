#include "MyViewer.h"
#include <math.h>

void MyViewer::inverse_kinematics(ControlPoint t, Join* j)
{
    ik.clear();
    tree_to_array(j);
    double dis = abs(distance(ik[0], t.position));
    
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
    FABRIK_p = ik;
    for (int i = 0; i < ik.size(); i++)
    {
        Join* s = j->searchbyid(j, i);
        s->point = ik[i];
    }
    IK_matrices(); 
    animate_mesh();
    skel.set_deafult_matrix();
    if (delatamush)
        Delta_Mush_two(vec);

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

    skel.po.clear();
    
    std::vector<Vec> old_p = skel.getPoints();
    for (int i = 1; i < n; ++i)
    {
        Join* t = skel.root->searchbyid(skel.root, i);


        Vec old_diff = old_p[i] - old_p[i - 1];
        Vec new_diff = ik[i] - ik[i - 1];
        old_diff = old_diff.unit();
        new_diff = new_diff.unit();
        Vec axis = old_diff ^ new_diff;
        float dot = old_diff.x * new_diff.x + old_diff.y * new_diff.y + old_diff.z * new_diff.z;
        float rotAngle = std::atan2(axis.norm(), dot); 

        Vec pivot = old_p[0];
        Mat4 T1 = TranslateMatrix(-pivot);
        Mat4 T2 = TranslateMatrix(pivot);

        Mat4 R;
        if (axis.norm() > 1E-12) {
            axis = axis.unit();
            //axis = Vec(0, 0, 1);
            R = RotationMatrix(rotAngle, axis);
            
        }
        Mat4 M = T1 * R * T2;
        Vec4 p = Vec4(t->Tpose) * M;
        t->M = M;
        t->point = Vec(p.x,p.y,p.z);
        

    }
    


}


void MyViewer::tree_to_array(Join* j)
{
    
    Vec& r = j->point;
    ik.push_back(r);
    for (int i = 0; i < j->children.size(); i++)
    {
        tree_to_array(j->children[i]);
    }
}