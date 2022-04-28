#pragma once

#include <QGLViewer/qglviewer.h>
using qglviewer::Vec;

struct Bones
{
    Vec start;
    Vec End;
    std::vector<Vec> points;
    double x, y, z;

    Vec getColor()
    {
        return Vec(x, y, z);
    }

    void setColor(double _x, double _y, double _z)
    {
        x = _x;
        y = _y;
        z = _z;
    }

    void manypoints()
    {

        Vec ir = End - start;
        double len = sqrt(pow(End.x - start.x, 2) + pow(End.y - start.y, 2) + pow(End.z - start.z, 2));
        int res = 100;
        for (int i = 0; i < res; i++)
        {
            Vec t;
            t = start + ir * 1.0 / (res-1) * i;
            points.push_back(t);
        }
    }
};








