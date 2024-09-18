#include "MyViewer.h"
#include <math.h>
#include <igl/Timer.h>
void MyViewer::inverse_kinematics(ControlPoint t, Joint* j)
{
    igl::Timer timer;
    timer.start();
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

    IK_matrices(j); 
    
    skel.animate_mesh(mesh,isweight,true);
    //j->set_deafult_matrix(j);

    timer.stop();
    double td = timer.getElapsedTimeInSec();
    if (delatamush)
    {
        //MushHelper = mesh;
        dm.setHelper(mesh);
        //dm.Delta_Mush_two(mesh);
        //AnDirectMush();
        MushHelper = mesh;
        Delta_Mush_two(vec);
    }
        

}




void MyViewer::IK_matrices(Joint*j)
{
    // FIX it for the j
    int n = ik.size();

    skel.po.clear();
    
    skel.joint.clear();

    // get joint
    std::vector<Vec> old_p = skel.getPoints(j);

    auto joints = skel.getJointtoList(j);




    for (int i = 1; i < n; ++i)
    {
        Joint* t = joints[i];


        Vec old_diff = old_p[i] - old_p[i - 1];
        Vec new_diff = ik[i] - ik[i - 1];
        old_diff = old_diff.unit();
        new_diff = new_diff.unit();
        Vec axis = old_diff ^ new_diff;
        float dot = old_diff.x * new_diff.x + old_diff.y * new_diff.y + old_diff.z * new_diff.z;
        float rotAngle = std::atan2(axis.norm(), dot); 

        Vec pivot = t->parent->Tpose;
        Mat4 T1 = TranslateMatrix(-pivot);
        Mat4 T2 = TranslateMatrix(pivot);

        Mat4 R;
        if (axis.norm() > 1E-12) {
            axis = axis.unit();
            //axis = Vec(0, 0, 1);
            R = RotationMatrix(rotAngle, axis);
            
        }
        t->R = R;
        Mat4 M = T1 * R * TranslateMatrix(t->parent->point);// * T2;
        Vec4 p = Vec4(t->Tpose) * M;
        //p = p * T1;
        //p = p * TranslateMatrix(t->parent->point);
        t->M = M;
        t->point = Vec(p.x,p.y,p.z);
        

    }
    


}


void MyViewer::tree_to_array(Joint* j)
{
    
    Vec& r = j->point;
    ik.push_back(r);
    for (int i = 0; i < j->children.size(); i++)
    {
        tree_to_array(j->children[i]);
    }
}