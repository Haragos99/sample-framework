#include "MyViewer.h"

bool MyViewer::openSkelton(const std::string& filename, bool update_view)
{
    std::shared_ptr<Skelton> skelton = std::make_shared<Skelton>();  
    skelton->loadFile(filename);
    skelton->build();
    objects.push_back(skelton);
    setupCamera();
    return true;
}

bool MyViewer::openBezier(const std::string& filename, bool update_view) {
    size_t n, m;
    try {
        std::ifstream f(filename.c_str());
        f.exceptions(std::ios::failbit | std::ios::badbit);
        f >> n >> m;
        degree[0] = n++; degree[1] = m++;
        control_points.resize(n * m);
        for (size_t i = 0, index = 0; i < n; ++i)
            for (size_t j = 0; j < m; ++j, ++index)
                f >> control_points[index][0] >> control_points[index][1] >> control_points[index][2];
    }
    catch (std::ifstream::failure&) {
        return false;
    }
    model_type = ModelType::BEZIER_SURFACE;
    last_filename = filename;
    if (update_view)
        setupCamera();
    return true;
}


bool MyViewer::openMesh(const std::string& filename, bool update_view) {

    std::shared_ptr<BaseMesh> mesh = std::make_shared<BaseMesh>(filename);
    bool sucsess = mesh->open();
    objects.push_back(mesh);
    setupCamera();
    return sucsess;
}

