#include "MyViewer.h"





void MyViewer::animate()
{
    if (animationDuration_ >=9.0)
    {
        isAnimating_ = false;
    }
    //rewrite this part
    //second
    if (isAnimating_)
    {
        float current_time = (currentTime() - startAnimationTime_) / 1000.0;
        if (0 < animationDuration_)
        {
            // Cheacking frame status
            size_t i = 0;
            //while (i < keyframes_.size() - 1 && currentTime() >= keyframes_[i + 1].time()) {
                //i++;
            //}
            const Keyframe& startKeyframe = keyframes_[i];
            const Keyframe& endKeyframe = keyframes_[i + 1];

            getallpoints(sk);
            std::vector<Vec> old = ve;
            ve.clear();
            sk.animaterotaion(sk, i, 1, sk.child[0].child[0].point);
            getallpoints(sk);
            std::vector<Vec> newp = ve;
            ve.clear();
            int des = -1;
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
            }
            newp.clear();
            old.clear();
            update();

        }
        animationDuration_+=1.1;
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
        Tree* to = sk.searchbyid(sk, selected_vertex);
        Keyframe k = Keyframe(sb->value(), to->point, angels);
        sk.Addframe(*to,k);
        keyframes_.push_back(k);
    }
}
void MyViewer::Frame()
{
    ang = Vec(0, 0, 0);
   // angels = Vec(0, 0, 0);
   // Rotate();
   // Reset();
    isAnimating_ = true;
    Keyframe k = Keyframe(0.0, sk.child[0].child[0].point,Vec(0,0,0));
    Keyframe k2 = Keyframe(10.0, sk.child[0].child[0].point,Vec(30, 0, 0));

    Keyframe k3 = Keyframe(0.0, sk.child[0].child[1].point, Vec(0, 0, 0));
    Keyframe k4 = Keyframe(10.0, sk.child[0].child[1].point, Vec(30, 0, 0));

    sk.child[0].child[0].Addframe(sk.child[0].child[0],k);
    sk.child[0].child[0].Addframe(sk.child[0].child[0], k2);

    sk.child[0].child[1].Addframe(sk.child[0].child[1], k3);
    sk.child[0].child[1].Addframe(sk.child[0].child[1], k4);

    keyframes_.push_back(k);
    keyframes_.push_back(k2);
   
    animationDuration_ = 0.0;
    //Invers();
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