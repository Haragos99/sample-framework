#pragma once
#include "MyViewer.h"
struct BSpline {
    size_t du;
    size_t dv;
    size_t nu;
    size_t nv;
    std::vector<double> knots;
    std::vector<Vec> control_points;

    void open(std::string& filname);

};
