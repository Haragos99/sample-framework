#include "MyViewer.h"


bool MyViewer::saveBone(const std::string& filename) {
    if (model_type != ModelType::SKELTON)
        return false;
    
    return skel.save(filename);
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
