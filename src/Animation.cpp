#include "MyViewer.h"





void MyViewer::animate()
{
    //TODO: finish this
    if (startAnimationTime_ < endanimation) {
        FrameSecond = startAnimationTime_;
        for (auto& cp : cps)
        {
            cp.animate(startAnimationTime_);
            Joint* j = skel.root->searchbyid(skel.root, cp.jointid);
            inverse_kinematics(cp, j);
        }
       // skel.animate(startAnimationTime_, mesh);
        //skel.set_deafult_matrix();
        startAnimationTime_ += 1;
    }
    else
    {
        isAnimating_ = false;
        //render.saveVideo();
        int s = render.sizeframes();
        stopAnimation();
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
        mesh.data(v).color = Vec(0, 0, 0);
    }
    update();
}