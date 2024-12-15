#pragma once
#include <QGLViewer/qglviewer.h>
using qglviewer::Vec;
using qglviewer::Quaternion;

struct Keyframe {
public:
    Keyframe(){}
    Keyframe(float time, int index, const Vec& angles)
        : time_(time), _index(index), angles_(angles) {}

    Keyframe(float time, int index, const Vec& angles, Quaternion& q)
        : time_(time), _index(index), angles_(angles), rotation(q){}

    Keyframe(float time, const Vec& position,int index)
        : time_(time),  position_(position), _index(index) {}
    Vec angles_;
    Vec position_;
    Quaternion rotation;
    float time() const { return time_; }
    const Vec& position() const { return position_; }
    const Vec& angeles() const { return angles_; }
    Vec selected_point;
    int _index;
private:
    float time_;


};