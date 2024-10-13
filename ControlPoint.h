#pragma once
#include <QGLViewer/qglviewer.h>
#include "Keyframe.h"

using qglviewer::Vec;

struct ControlPoint {
    Vec position;
    Vec color;
    int jointid;
    int id;
    std::vector<Keyframe> keyframes;

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
    void animate(float t);
    void draw();
};