#include "MyViewer.h"


void MyViewer::inverse_kinematics(ControlPoint t, Tree& tree)
{
	double dis = abs(distance(tree.point, t.position));
    tree_to_array(tree);
    int start_index = tree.id;
    std::vector<double> distances;
    double total_distance = 0.0;
    //ik where we store the old points for the calculation of the ik
    for (int i = 0; i < ik.size() - 1; i++)
    {
        double d = abs(distance(ik[i], ik[i + 1]));
        distances.push_back(d);

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
            for (int i = ik.size() - 1; i > 0; i--)
            {
                double d = distances[i];
                double r = abs(distance(ik[i], ik[i + 1]));
                double alfa = d / r;
                ik[i] = (1 - alfa) * ik[i+1] + alfa * ik[i];

            }
            ik[0] = b;
            for (int i = 0; i < ik.size() - 1; i++)
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
    int j = 0;
    for (int i = start_index; i < points.size(); i++)
    {
        points[i] = ik[j];
        j++;
    }
    ininitSkelton();
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