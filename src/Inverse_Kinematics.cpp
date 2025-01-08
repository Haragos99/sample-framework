#include "MyViewer.h"
#include <math.h>
#include <igl/Timer.h>
void MyViewer::inverse_kinematics(ControlPoint t, Joint* j)
{
    igl::Timer timer;

    if (delatamush)
    {
        //MushHelper = mesh;
        
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