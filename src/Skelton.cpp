#include "Skelton.h"




void Skelton::animate(float current_time, MyMesh& mesh)
{

    for (int k = 0; k < n_joint; k++)
    {
        Joint* j = root->searchbyid(root, k);
        if (j->keyframes.size() != 0 && j->keyframes.back().time() >= current_time)
        {
            for (size_t i = 0; i < j->keyframes.size() - 1; ++i) {

                if (current_time >= j->keyframes[i].time() && current_time <= j->keyframes[i + 1].time())
                {
                    Vec pivot = j->point;
                    Vec rotated = (j->keyframes[i + 1].angeles() - j->keyframes[i].angeles());
                    float timediff = (j->keyframes[i + 1].time() - j->keyframes[i].time());
                    float step = 1;
                    float rate = timediff * step;
                    Vec angels = rotated / rate;
                    j->calculateMatrecies(j, pivot, angels);
                }
            }
        }

    }
    root->transform_point(root);
    animate_mesh(mesh, true);
}



void Skelton::animate_mesh(MyMesh& mesh, bool isweight, bool inv)
{
    if (isweight)
    {
        for (auto v : mesh.vertices())
        {
            // tezstként lehet leutánozni a fabrikot
            Mat4 M_result = Mat4(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
            for (int i = 0; i < bones.size(); i++)
            {
                double w = mesh.data(v).weigh[i];
                Mat4 M = bones[i].end->M.skalar(w);
                M_result += M;
            }
            Vec4 point4;
            //origanal részt újra gondolni
            if (!inv)
            {
                point4 = Vec4(mesh.point(v)[0], mesh.point(v)[1], mesh.point(v)[2], 1);
            }
            else {
                point4 = Vec4(mesh.data(v).original[0], mesh.data(v).original[1], mesh.data(v).original[2], 1);
            }

            Vec4 result = point4 * M_result;
            OpenMesh::Vec3d diffrents = OpenMesh::Vec3d(result.x, result.y, result.z);
            mesh.point(v) = diffrents;
            mesh.data(v).M = M_result;
        }
    }
}



void Skelton::addJoint(Joint* parent, Joint* child) {
    if (parent == nullptr)
    {
        root = child; // If no parent, child becomes root
    }
    else {
        parent->children.push_back(child);
        child->parent = parent;
    }
}




void Skelton::buildTree(std::vector<Joint*>& joints)
{

    for (int i = 0; i < childrenMatrix.size(); ++i)
    {
        for (int j = 0; j < childrenMatrix[i].size(); ++j)
        {
            int childId = childrenMatrix[i][j];
            if (childId != -1) // -1 indicates no child
            {
                Joint* parent = joints[i];
                Joint* child = joints[childId];
                addJoint(parent, child);
            }
        }
    }
}


//TODO: Rewrite
void Skelton::calculateMatrix()
{

}


void Skelton::loadFile(const std::string& filename)
{
    std::ifstream file(filename); // Replace "data.txt" with your file name

    std::vector<int> children;


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
            indexes.push_back({ idx1, idx2 });
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
            childrenMatrix.push_back(children);
            children.clear();
        }
    }

    file.close();

}

void Skelton::build()
{

    std::vector<Joint*> joints;
    for (int i = 0; i < points.size(); i++)
    {
        joints.push_back(new Joint(points[i], i));
    }
    n_joint = joints.size();

    //Build tree by BFS

    root = joints[0];


    buildTree(joints);


    // build the bones
    int size = indexes.size();
    for (int i = 0; i < size; i++)
    {
        bones.push_back(Bone(joints[indexes[i].first], joints[indexes[i].second], i, colors_bone[i]));

    }

}

std::vector<Axes>Skelton::arrows()
{
    std::vector<Axes> result;
    for (int i = 0; i < n_joint; i++)
    {
        Joint* j = root->searchbyid(root, i);
        result.push_back(Axes(j->point, 0.1, j->R));
    }
    return result;
}


void Skelton::setJointMatrix(int id, Vec& angle)
{
    Joint* j = root->searchbyid(root, id);
    j->setMatrix(angle);
}

bool Skelton::save(const std::string& filename)
{
    try {

        std::ofstream f(filename.c_str());
        f.exceptions(std::ios::failbit | std::ios::badbit);
        for (int i = 0; i < n_joint; i++)
        {
            Joint* j = root->searchbyid(root, i);
            Vec p = j->point;
            f << 'b' << ';' << p[0] << ';' << p[1] << ';' << p[2] << ';' << std::endl;
        }
        for (const auto& i : indexes)
        {
            f << 'a' << ';' << i.first << ';' << i.second << ';' << std::endl;
        }

        for (const auto& m : childrenMatrix)
        {
            std::string line = "i;";
            line = line + std::to_string(m.size()) + ";";
            for (int i : m)
            {
                line = line + std::to_string(i) + ";";
            }
            f << line << std::endl;
        }


    }
    catch (std::ifstream::failure&) {
        return false;
    }


    return true;
}

