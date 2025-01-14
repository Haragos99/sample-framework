#pragma once
#include "PoissonResurses.h"

class PoissonSampleGenerator
{
public:
    std::vector<MyMesh::Point> poissonDisk(float radius, std::vector<SamplePoint> raw, std::vector<MyMesh::Normal>& samples_nor);
    float generateSamples(int num_samples, MyMesh mesh_, std::vector<SamplePoint>& samples);

private:
        float random_float(float maximum) { return float((double)rand() / (double)(RAND_MAX / maximum)); }
        float approximate_geodesic_distance(MyMesh::Point p1, MyMesh::Point p2, MyMesh::Normal n1, MyMesh::Normal  n2);
};
