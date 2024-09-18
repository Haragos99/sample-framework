#pragma once
#include <QGLViewer/qglviewer.h>
using qglviewer::Vec;


struct Keyframe {
public:
    Keyframe(){}
    Keyframe(float time, int index, const Vec& angles)
        : time_(time), index_of_join(index), angles_(angles) {}
    Vec angles_;
    Vec position_;
    float time() const { return time_; }
    const Vec& position() const { return position_; }
    const Vec& angeles() const { return angles_; }
    Vec selected_point;
    int index_of_join;
private:
    float time_;


};