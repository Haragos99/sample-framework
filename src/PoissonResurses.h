#pragma once
#include "Mesh.h"


struct Idx3 {

    MyMesh::Point _size;
    int id;

    Idx3(const MyMesh::Point& size, int idx) : _size(size), id(idx) { }

    Idx3(const MyMesh::Point& size, const MyMesh::Point& pos) : _size(size) {
        set_3d(pos);
    }

    void set_3d(const MyMesh::Point& pos) { id = to_linear(_size, pos[0], pos[1], pos[2]); }


    int to_linear(const MyMesh::Point& size_, int x, int y, int z) {
        return x + size_[0] * (y + size_[1] * z);
    }

    int to_linear() const { return id; }
};


struct BBox {
    MyMesh::Point min, max;
    BBox()
    {
        min = MyMesh::Point(FLT_MAX, FLT_MAX, FLT_MAX);
        max = MyMesh::Point(-FLT_MAX, -FLT_MAX, -FLT_MAX);
    }

    void add_point(MyMesh::Point& p)
    {
        min[0] = fminf(p[0], min[0]);
        min[1] = fminf(p[1], min[1]);
        min[2] = fminf(p[2], min[2]);
        max[0] = fmaxf(p[0], max[0]);
        max[1] = fmaxf(p[1], max[1]);
        max[2] = fmaxf(p[2], max[2]);
    }
    MyMesh::Point lenghts() { return max - min; }
    MyMesh::Point index_grid_cell(MyMesh::Point res, MyMesh::Point p)
    {
        MyMesh::Point cell_l = lenghts() / res;

        MyMesh::Point lcl = p - min;
        MyMesh::Point idx = lcl / cell_l;
        MyMesh::Point int_idx = MyMesh::Point((int)floorf(idx[0]), (int)floorf(idx[1]), (int)floorf(idx[2]));
        return int_idx;
    }

};

struct poisson_sample {
    MyMesh::Point pos;
    MyMesh::Normal normal;
};

struct hash_data {
    // Resulting output sample points for this cell:
    std::vector<poisson_sample> poisson_samples;

    // Index into raw_samples:
    int first_sample_idx;
    int sample_cnt;
};


struct SamplePoint {
    MyMesh::FaceHandle tri;
    int cell_id;
    MyMesh::Point pos;
    MyMesh::Normal normal;

    SamplePoint(MyMesh::Point _pos, MyMesh::Normal _normal) { pos = _pos; normal = _normal; }
};
