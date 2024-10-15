#include "MyViewer.h"





void MyViewer::animate()
{
    //TODO: finish this
    if (startAnimationTime_ < endanimation) {
        FrameSecond = startAnimationTime_;
        for (auto& cp : cps)
        {
            //cp.animate(startAnimationTime_);
            //inverse_kinematics(cp,skel.root);
            cp.animate(startAnimationTime_);


            //cp.position = (qreal)(1.0f - startAnimationTime_) * cp.position + (qreal)startAnimationTime_ * Vec(1, 1, 1);
            inverse_kinematics(cp, skel.root);
        }
        startAnimationTime_ += 0.01;
    }
    else
    {
        isAnimating_ = false;
        render.saveVideo();
        int s = render.sizeframes();
        stopAnimation();
    }
    
    if (isAnimating_&& false)
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


        if (model_type == ModelType::INVERZ)
        {
            Keyframe k = Keyframe(sb->value(), cps[selected_vertex].position,cps[selected_vertex].id);
            cps[selected_vertex].addkeyframe(k);
            keyframes_.push_back(k);
        }
        else
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
}
void MyViewer::Frame()
{
    ang = Vec(0, 0, 0);
    isAnimating_ = true;
    animationDuration_ = 10.0;
    startAnimationTime_ = currentTime();
    startAnimationTime_ = 0;
    endanimation = keyframes_.back().time();
    //Invers();
    Reset();
    startAnimation();

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