#include "Bone.h"



void Bone::draw()
{
    glLineWidth(200.0);
    glBegin(GL_LINES);
    glColor3d(color.x, color.y, color.z);
    glVertex3dv(start->point);
    glVertex3dv(end->point);
    glEnd();
}


void Bone::manypoints()
{

    Vec ir = end->point - start->point;
    double len = sqrt(pow(end->point.x - start->point.x, 2) +
        pow(end->point.y - start->point.y, 2) + pow(end->point.z - start->point.z, 2));
    int res = 100;
    for (int i = 0; i < res; i++)
    {
        Vec t;
        t = start->point + ir * 1.0 / (res - 1) * i;
        points.push_back(t);
    }
}




void Skelton::animate(float current_time, MyMesh& mesh)
{
    //root->animaterotaion(root, current_time, Mat4());
    for (int i = 0; i < 4; i++)
    {
        Joint* j = root->searchbyid(root, i);
        if (j->keyframes.size() != 0 && j->keyframes.back().time() >= current_time)
        {
            size_t s = 0;

            // think again -2 
            while (s < j->keyframes.size() - 2 && current_time >= j->keyframes[s + 1].time()) {
                s++;
            }
            const Keyframe& startKeyframe = j->keyframes[s];
            const Keyframe& endKeyframe = j->keyframes[s + 1];
            float timediff_key = (endKeyframe.time() - startKeyframe.time());
            float dt = current_time - startKeyframe.time();
            Vec rotated = (endKeyframe.angeles() - startKeyframe.angeles());
            float r = (dt / timediff_key);
            Vec angels = rotated * r / 10;
            if (dt >= endKeyframe.time())angels = Vec(0, 0, 0);
            Vec pivot = j->point;
            j->calculateMatrecies(j, pivot, angels);


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
    int id = 0;
    for (int i = 0; i < size; i++)
    {
        bones.push_back(Bone(joints[indexes[i].first], joints[indexes[i].second], id,colors_bone[i]));
        id++;
    }

}

std::vector<Axes>Skelton::arrows()
{
    std::vector<Axes> result;
    for (int i = 0; i < n_joint; i++)
    {
        Joint* j = root->searchbyid(root, i);
        result.push_back(Axes(j->point, 0.1,j->R));
    }
    return result;
}


bool Skelton::save(const std::string& filename)
{
    try {

        std::ofstream f(filename.c_str());
        f.exceptions(std::ios::failbit | std::ios::badbit);
        for (int i = 0; i < n_joint;i++)
        {
            Joint* j = root->searchbyid(root, i);
            Vec p = j->point;
            f << 'b' << ';' << p[0] << ';' << p[1] << ';' << p[2] << ';' << std::endl;
        }
        for (const auto& i : indexes)
        {
            f<< 'a' << ';'<< i.first << ';' << i.second<<';' << std::endl;
        }

        for (const auto& m : childrenMatrix)
        {
            std::string line ="i;";
            line = line + std::to_string(m.size())+";";
            for (int i : m)
            {
                line = line + std::to_string(i) + ";";
            }
            f<<line<< std::endl;
        }


    }
    catch (std::ifstream::failure&) {
        return false;
    }


    return true;
}

