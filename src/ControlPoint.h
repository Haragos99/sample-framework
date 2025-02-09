#pragma once
#include <QGLViewer/qglviewer.h>
#include "Keyframe.h"
#include "Object3D.h"
#include "skeleton/Joint.h"
#include "InverseKinematics.h"
using qglviewer::Vec;

class ControlPoint : public  Object3D {

public:
    Vec position_;
    Vec color;
    int jointid;
    int id;
    Joint* joint;
    ControlPoint() = default;
    ControlPoint(Vec _position);
    ControlPoint(Vec _position, int _id);
    ControlPoint(Vec _position, int _id, std::shared_ptr<Skelton> skelton);
    void drawarrow();
    void addkeyframe(Keyframe& k) { keyframes.push_back(k); }
    void animate(float time) override;
    void draw();
    void drawWithNames(Vis::Visualization& vis) const override;
    Vec postSelection(const int p) override;
    void draw(Vis::Visualization& vis) override;
    void scale(float scale) override;
    void movement(int selected, const Vector& position) override;
    void rotate(int selected, Vec angel) override;
    void setCameraFocus(Vector& min, Vector& max) override;
    void addKeyframes(int selected,float timeline) override;
    void reset()override;
    void datainfo() override;
    void inversekinematics(std::shared_ptr<Skelton> skelton);
    ~ControlPoint() = default;
private:
    
    std::unique_ptr<InverseKinematics> IK;

};