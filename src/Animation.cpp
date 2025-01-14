#include "MyViewer.h"

void MyViewer::animate()
{
    //TODO: finish this
    if (startAnimationTime_ < endanimation) {
        FrameSecond = startAnimationTime_;
        for (auto object : objects)
        {
            object->animate(startAnimationTime_);
        }
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

float MyViewer::currentTime() {
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::duration<float>>(duration).count();
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
        Keyframe k = Keyframe(sb->value(), 0, angels);
        objects[selected_object]->addKeyframes(selected_vertex,sb->value());
        keyframes_.push_back(k);
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
    Reset();
    startAnimation();

}

void MyViewer::Reset()
{
    for (auto object : objects)
    {
        object->reset();
    }
    update();
}