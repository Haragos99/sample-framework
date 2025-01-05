#pragma once
#include <QGLViewer/qglviewer.h>
#include "Keyframe.h"
#include "Object3D.h"

using qglviewer::Vec;

struct ControlPoint :Object3D {
    Vec position;
    Vec color;
    int jointid;
    int id;

    ControlPoint() {}
    ControlPoint(Vec _position)
    {
        position = _position;
        color = Vec(1, 0, 0);

    }
    ControlPoint(Vec _position, int _id)
    {
        position = _position;
        color = Vec(1, 0, 0);
        id = _id;
    }
    Vec interpolatedPosition(float t);
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


};