#include "MyViewer.h"





void MyViewer::animate()
{
   
    if (isAnimating_)
    {
        float current_time = (currentTime() - startAnimationTime_) * 10;
        FrameSecond = current_time;
        update();
        if (current_time < animationDuration_)
        {
            skel.animate(current_time,mesh);
            //animate_mesh(); // animate the mesh
            skel.set_deafult_matrix();
            update();

        }
        else {
            isAnimating_ = false;
            stopAnimation();
        }
    }
    
     
}





void MyViewer::keyframe_add()
{
    auto dlg = std::make_unique<QDialog>(this);
    auto* hb1 = new QHBoxLayout;
    auto* hb2 = new QHBoxLayout;
    auto* vb = new QVBoxLayout;

    auto* text_H = new QLabel(tr("Save the point Keyframe"));
    auto* sb = new QDoubleSpinBox;
    auto* cancel = new QPushButton(tr("Cancel"));
    auto* ok = new QPushButton(tr("Ok"));
    connect(cancel, SIGNAL(pressed()), dlg.get(), SLOT(reject()));
    connect(ok, SIGNAL(pressed()), dlg.get(), SLOT(accept()));
    ok->setDefault(true);
    sb->setRange(-360, 360);;
    hb1->addWidget(cancel);
    hb1->addWidget(ok);
    hb2->addWidget(text_H);
    hb2->addWidget(sb);
    vb->addLayout(hb2);
    vb->addLayout(hb1);
    dlg->setWindowTitle(tr("KeyFrame"));
    dlg->setLayout(vb);

    if (dlg->exec() == QDialog::Accepted) 
    {
        qglviewer::Quaternion qx = qglviewer::Quaternion(Vec(1, 0, 0), angels.x / 180.0 * M_PI);
        qglviewer::Quaternion qy = qglviewer::Quaternion(Vec(0, 1, 0), angels.y / 180.0 * M_PI);
        qglviewer::Quaternion qz = qglviewer::Quaternion(Vec(0, 0, 1), angels.z / 180.0 * M_PI);
        qglviewer::Quaternion q = qz * qy * qx;

        Joint* j = skel.root->searchbyid(skel.root, selected_vertex);
        Keyframe k = Keyframe(sb->value(), j->id, angels);

        skel.root->addframe(j, k);
        keyframes_.push_back(k);
    }
}
void MyViewer::Frame()
{
    ang = Vec(0, 0, 0);
    isAnimating_ = true;
    animationDuration_ = 10.0;
    startAnimationTime_ = currentTime();
    //Invers();
    Reset();
    startAnimation();

}





void MyViewer::Invers()
{

    Tree* to = sk.searchbyid(sk, selected_vertex);
    //to->endframe = end.endframe;
    getallpoints(*to);
    std::vector<Vec> old = selected_points_storage;
    selected_points_storage.clear();
    sk.change_all_rotason(*to, to->point, -angels);
    getallpoints(*to);
    std::vector<Vec> newp = selected_points_storage;
    selected_points_storage.clear();
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
    skel.reset();



    for (auto v : mesh.vertices())
    {
        mesh.point(v)= mesh.data(v).original;
    }
    update();
}