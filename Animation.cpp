#include "MyViewer.h"

void MyViewer::animate()
{
    
    //rewrite this part
    //second
    if (isAnimating_)
    {
        float current_time = (currentTime() - startAnimationTime_) / 1000.0;
        if (current_time < animationDuration_)
        {
            // Cheacking frame status
            size_t i = 0;
            while (i < keyframes_.size() - 1 && currentTime() >= keyframes_[i + 1].time()) {
                i++;
            }
            const Keyframe& startKeyframe = keyframes_[i];
            const Keyframe& endKeyframe = keyframes_[i + 1];

        }
    }


    //-----------------------------------------------------
    if (ang.x <= angels.x )
    {
        Tree* to = sk.searchbyid(sk, selected_vertex);
        //to->endframe = end.endframe;
        getallpoints(*to);
        std::vector<Vec> old = ve;
        ve.clear();

        double x = 0;
        double y = 0;
        double z = 0;
        // sk.animatepoziton(*to);
        // sk.animaterotaion(*to);
            

        if (angels.x > 0) x = 1;
        if (angels.y > 0) y = 1;
        if (angels.z > 0) z = 1;

            ang.x += 1;
            sk.change_all_rotason(*to, to->point, Vec(x,y,z));
            getallpoints(*to);
            std::vector<Vec> newp = ve;
            ve.clear();
            int des = -1;
            //double d = 0;
            for (int i = 0; i < b.size(); i++)
            {
                for (int j = 0; j < old.size(); j++)
                {
                    if (b[i].start == old[j])
                    {
                        b[i].start = newp[j];
                        des = i;
                    }
                    if (b[i].End == old[j])
                    {
                        b[i].End = newp[j];
                        //  des = i;
                    }
                }
                if (des != -1)
                {
                    if (isweight)
                    {
                        for (auto v : mesh.vertices())
                        {
                            qglviewer::Quaternion qx = qglviewer::Quaternion(Vec(1, 0, 0), (x * mesh.data(v).weigh[des]) / 180.0 * M_PI);
                            qglviewer::Quaternion qy = qglviewer::Quaternion(Vec(0, 1, 0), (y * mesh.data(v).weigh[des]) / 180.0 * M_PI);
                            qglviewer::Quaternion qz = qglviewer::Quaternion(Vec(0, 0, 1), (z * mesh.data(v).weigh[des]) / 180.0 * M_PI);


                                Vec p = Vec(mesh.point(v)[0], mesh.point(v)[1], mesh.point(v)[2]);
                                Vec result = to->point + qx.rotate(p- to->point);
                                OpenMesh::Vec3d diffrents = OpenMesh::Vec3d(result.x, result.y, result.z);
                                mesh.point(v) = diffrents;

                      
                                p = Vec(mesh.point(v)[0], mesh.point(v)[1], mesh.point(v)[2]);
                                Vec result2 = to->point + qy.rotate(p - to->point);
                                OpenMesh::Vec3d diffrents2 = OpenMesh::Vec3d(result2.x, result2.y, result2.z);      
                                mesh.point(v) = diffrents2;

                        
                                p = Vec(mesh.point(v)[0], mesh.point(v)[1], mesh.point(v)[2]);
                                Vec result3 = to->point + qz.rotate(p - to->point);
                                OpenMesh::Vec3d diffrents3 = OpenMesh::Vec3d(result3.x, result3.y, result3.z);
                                mesh.point(v) = diffrents3;

                        

                        }
                        des = -1;
                    }
                }
            
            }
            newp.clear();
            old.clear();
            update();
        }
    else
    {
        angels = Vec(0, 0, 0);
        stopAnimation();
    }

}




void MyViewer::Frame()
{
    ang = Vec(0, 0, 0);
   // angels = Vec(0, 0, 0);
   // Rotate();
   // Reset();
    Invers();
    startAnimation();

}





void move(std::vector<Vec> newp, std::vector<Vec> old) 
{

}



void MyViewer::Invers()
{

    Tree* to = sk.searchbyid(sk, selected_vertex);
    //to->endframe = end.endframe;
    getallpoints(*to);
    std::vector<Vec> old = ve;
    ve.clear();
    sk.change_all_rotason(*to, to->point, -angels);
    getallpoints(*to);
    std::vector<Vec> newp = ve;
    ve.clear();
    int des = -1;
    //double d = 0;
    for (int i = 0; i < b.size(); i++)
    {
        for (int j = 0; j < old.size(); j++)
        {
            if (b[i].start == old[j])
            {
                b[i].start = newp[j];
                des = i;
            }
            if (b[i].End == old[j])
            {
                b[i].End = newp[j];
                //  des = i;
            }
        }
        if (des != -1)
        {
            if (isweight)
            {
                for (auto v : mesh.vertices())
                {
                    qglviewer::Quaternion qx = qglviewer::Quaternion(Vec(1, 0, 0), (-angels.x * mesh.data(v).weigh[des]) / 180.0 * M_PI);
                    qglviewer::Quaternion qy = qglviewer::Quaternion(Vec(0, 1, 0), (-angels.y * mesh.data(v).weigh[des]) / 180.0 * M_PI);
                    qglviewer::Quaternion qz = qglviewer::Quaternion(Vec(0, 0, 1), (-angels.z * mesh.data(v).weigh[des]) / 180.0 * M_PI);


                    Vec p = Vec(mesh.point(v)[0], mesh.point(v)[1], mesh.point(v)[2]);
                    Vec result = to->point + qx.rotate(p - to->point);
                    OpenMesh::Vec3d diffrents = OpenMesh::Vec3d(result.x, result.y, result.z);
                    mesh.point(v) = diffrents;


                    p = Vec(mesh.point(v)[0], mesh.point(v)[1], mesh.point(v)[2]);
                    Vec result2 = to->point + qy.rotate(p - to->point);
                    OpenMesh::Vec3d diffrents2 = OpenMesh::Vec3d(result2.x, result2.y, result2.z);
                    mesh.point(v) = diffrents2;


                    p = Vec(mesh.point(v)[0], mesh.point(v)[1], mesh.point(v)[2]);
                    Vec result3 = to->point + qz.rotate(p - to->point);
                    OpenMesh::Vec3d diffrents3 = OpenMesh::Vec3d(result3.x, result3.y, result3.z);
                    mesh.point(v) = diffrents3;



                }
                des = -1;
            }
        }


    }
    newp.clear();
    old.clear();
    update();
}

void MyViewer::Reset()
{
    sk.reset_all(sk);
    for (int i = 0; i < b.size(); i++)
    {
        b[i].start = b[i].originalS;
        b[i].End = b[i].originalE;
    }

    for (auto v : mesh.vertices())
    {
        mesh.point(v)= mesh.data(v).original;
    }
    update();
}