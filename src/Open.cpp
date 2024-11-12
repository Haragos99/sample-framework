#include "MyViewer.h"




bool MyViewer::openSkelton(const std::string& filename, bool update_view)
{
    
    indexes.clear();
    b.clear();
    points.clear();
    show_skelton = true;

   

    skel = Skelton();
    skel.loadFile(filename);
    skel.build();
    model_type = ModelType::SKELTON;
    last_filename = filename;
    points = skel.getPointlist();
    //target = ControlPoint(points.back());
    //target.position *= 1.1;

    updateMesh(update_view);
    if (update_view)
        setupCameraBone();

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
    updateMesh(update_view);
    if (update_view)
        setupCamera();
    return true;
}


bool MyViewer::openMesh(const std::string& filename, bool update_view) {
    if (!OpenMesh::IO::read_mesh(mesh, filename) || mesh.n_vertices() == 0)
        return false;
    model_type = ModelType::MESH;
    last_filename = filename;
    updateMesh(update_view);

    if (update_view)
        setupCamera();
    
    for (auto v : mesh.vertices())
    {
        mesh.data(v).original = mesh.point(v);
    }
    mesh.request_vertex_status();
    mesh.request_edge_status();
    mesh.request_face_status();
    

    return true;
}

