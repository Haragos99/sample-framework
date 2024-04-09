#include "MyViewer.h"




bool MyViewer::openSkelton(const std::string& filename, bool update_view)
{
    Tree t;
    sk = t;
    indexes.clear();
    b.clear();
    points.clear();
    if (filename.find("fac") != std::string::npos) {
        skellton_type = SkelltonType::FACE;

    }

    else if (filename.find("csuk") != std::string::npos) {
        skellton_type = SkelltonType::WRIST;

    }
    else if (filename.find("arm") != std::string::npos)
    {
        skellton_type = SkelltonType::ARM;
    }
    else if (filename.find("man") != std::string::npos)
    {
        skellton_type = SkelltonType::MAN;
    }
    else
    {
        return false;
    }
    show_skelton = true;

    std::ifstream file(filename); // Replace "data.txt" with your file name

    std::vector< std::pair<int, int>> indices;
    std::vector<int> children;
    std::vector< std::vector<int>> childrenIndices;

    std::string line;
    while (getline(file, line)) {
        std::stringstream ss(line);
        char type;
        ss >> type;

        if (type == 'b') {
            char dummy;
            double x, y, z;
            ss >> dummy >> x >> dummy >> y >> dummy >> z >> dummy;
            points.push_back({ x, y, z });
        }
        else if (type == 'a') {
            char dummy;
            int idx1, idx2;
            ss >> dummy >> idx1 >> dummy >> idx2 >> dummy;
            indices.push_back({ idx1, idx2 });
        }
        else if (type == 'i') {
            char dummy;
            int idx;
            int size;
            ss >> dummy >> size;
            for (int i = 0; i < size; i++)
            {
                ss >> dummy >> idx;
                children.push_back(idx);
            }
            childrenIndices.push_back(children);
            children.clear();
        }
    }

    file.close();


    skel = Skelton(points, childrenIndices, indices);
    skel.buildjoint();
    model_type = ModelType::SKELTON;
    last_filename = filename;
    //ininitSkelton();
    target = ControlPoint(points.back());
    target.position *= 1.1;

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

void MyViewer::ininitSkelton()
{
    b.clear();
    int size = indexes.size();
    for (int i = 0; i < size; i += 2)
    {
        // ide kelle majd
        Bones bo;
        bo.start = points[indexes[i] - 1];
        bo.End = points[indexes[i + 1] - 1];
        bo.originalS = bo.start;
        bo.originalE = bo.End;
        bo.lenght = distance(bo.start, bo.End);
        b.push_back(bo);

        
    }

    for (int i = 0; i < b.size(); i++)
    {
        b[i].setColor(colors_bone[i].x, colors_bone[i].y, colors_bone[i].z);
        b[i].manypoints();
    }

    if (skellton_type == SkelltonType::MAN) {
        manSkellton();
    }
    if (skellton_type == SkelltonType::WRIST) {
        csukloSkellton();
    }
    if (skellton_type == SkelltonType::ARM)
    {
        armSkellton();
    }
    if (skellton_type == SkelltonType::FACE)
    {
        faceSkellton();
    }
    FABRIK = sk;
}

