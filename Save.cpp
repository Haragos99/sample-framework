#include "MyViewer.h"


bool MyViewer::saveBone(const std::string& filename) {
    if (model_type != ModelType::SKELTON)
        return false;

    try {
        getallpoints(sk);
        points = selected_points_storage;
        selected_points_storage.clear();
        std::ofstream f(filename.c_str());
        f.exceptions(std::ios::failbit | std::ios::badbit);
        for (const auto& p : points)
            f << 'b' << ';' << p[0] << ';' << p[1] << ';' << p[2] << ';' << std::endl;
        f << '#' << std::endl;
        for (int i = 0; i < indexes.size(); i += 2)
        {
            f << indexes[i] << ';' << indexes[i + 1] << ';' << std::endl;
        }
        f << '#' << std::endl;
    }
    catch (std::ifstream::failure&) {
        return false;
    }
    return true;
}


bool MyViewer::saveBezier(const std::string& filename) {
    if (model_type != ModelType::BEZIER_SURFACE)
        return false;

    try {
        std::ofstream f(filename.c_str());
        f.exceptions(std::ios::failbit | std::ios::badbit);
        f << degree[0] << ' ' << degree[1] << std::endl;
        for (const auto& p : control_points)
            f << p[0] << ' ' << p[1] << ' ' << p[2] << std::endl;
    }
    catch (std::ifstream::failure&) {
        return false;
    }
    return true;
}
