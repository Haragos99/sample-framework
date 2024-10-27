#pragma once
#include <algorithm>
#include <limits>

//an Axis - Aligned Bounding Box - AABB 
struct AABB {
    float min[3];  // Minimum x, y, z coordinates (lower corner of the box)
    float max[3];  // Maximum x, y, z coordinates (upper corner of the box)

    // Constructor to reset AABB to an invalid state
    AABB() {
        reset();
    }

    // Initialize AABB to an invalid state (so it can be expanded later)
    void reset() {
        min[0] = min[1] = min[2] = std::numeric_limits<float>::max();
        max[0] = max[1] = max[2] = std::numeric_limits<float>::lowest();
    }

    // Expand the current AABB to include another AABB
    void expand(const AABB& other) {
        for (int i = 0; i < 3; ++i) {
            min[i] = std::min(min[i], other.min[i]);
            max[i] = std::max(max[i], other.max[i]);
        }
    }

    // Expand the current AABB to include a point (e.g., a vertex)
    void expand(const double* point) {
        for (int i = 0; i < 3; ++i) {
            min[i] = std::min(min[i], (float)point[i]);
            max[i] = std::max(max[i], (float)point[i]);
        }
    }


    void expand(const float point[3]) {
        for (int i = 0; i < 3; ++i) {
            min[i] = std::min(min[i], point[i]);
            max[i] = std::max(max[i], point[i]);
        }
    }





    // Check if this AABB overlaps with another AABB
    bool overlaps(const AABB& other) const {
        for (int i = 0; i < 3; ++i) {
            if (max[i] < other.min[i] || min[i] > other.max[i]) {
                return false;
            }
        }
        return true;
    }

    // Check if a point is inside this AABB
    bool contains(const float point[3]) const {
        for (int i = 0; i < 3; ++i) {
            if (point[i] < min[i] || point[i] > max[i]) {
                return false;
            }
        }
        return true;
    }
};
