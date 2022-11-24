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
    std::string myText;
    std::vector<double> linepoint;
    // Read from the text file
    std::ifstream MyReadFile(filename);
    bool isindex = false;
    // Use a while loop together with the getline() function to read the file line by line
    while (std::getline(MyReadFile, myText)) {
        size_t pos = 0;
        std::string delimiter = ";";
        std::string token;

        if (myText != "#")
        {
            while ((pos = myText.find(delimiter)) != std::string::npos) {
                token = myText.substr(0, pos);
                std::cout << token << " ";
                if (isindex)
                {
                    indexes.push_back(std::stoi(token));
                }
                if (pos != 1 && !isindex)
                {
                    linepoint.push_back(std::stod(token));
                }
                myText.erase(0, pos + delimiter.length());
            }
            std::cout << '\n';
        }
        else {
            std::cout << myText << '\n';
            isindex = true;
        }
        if (!isindex)
        {
            points.push_back(Vec(linepoint[0], linepoint[1], linepoint[2]));
            linepoint.clear();
        }
    }

    // Close the file
    MyReadFile.close();
    model_type = ModelType::SKELTON;
    last_filename = filename;
    ininitSkelton();
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

    return true;
}

void MyViewer::ininitSkelton()
{

    int size = indexes.size();
    for (int i = 0; i < size; i += 2)
    {
        Bones bo;
        bo.start = points[indexes[i] - 1];
        bo.End = points[indexes[i + 1] - 1];
        bo.originalS = bo.start;
        bo.originalE = bo.End;
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
    start = sk;

}

