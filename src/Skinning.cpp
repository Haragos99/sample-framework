#include "Skinning.h"





void Skinning::clean(MyMesh& mesh, std::vector<Bone>& bones)
{
    for (auto v : mesh.vertices())
    {
        mesh.data(v).weigh.clear();
        mesh.data(v).weigh.resize(bones.size(), 0.0);
        mesh.data(v).distance.clear();
        mesh.data(v).distance.resize(bones.size(), std::numeric_limits<double>::infinity());
    }

}



void Skinning::animatemesh(std::shared_ptr<BaseMesh> basemesh, std::vector<Bone>& bones)
{
    MyMesh& mesh = basemesh->getMesh();
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
        if (true)
        {
            point4 = Vec4(mesh.point(v)[0], mesh.point(v)[1], mesh.point(v)[2], 1);
        }
        else {
            point4 = Vec4(mesh.data(v).original[0], mesh.data(v).original[1], mesh.data(v).original[2], 1);
        }

        Vec4 result = point4 * M_result;
        OpenMesh::Vec3d newposition = OpenMesh::Vec3d(result.x, result.y, result.z);
        mesh.point(v) = newposition;
        mesh.data(v).M = M_result;
    }
}

void Skinning::execute(std::shared_ptr<BaseMesh> basemesh, std::vector<Bone>& bones)
{
    MyMesh& mesh = basemesh->getMesh();
    calculateSkinning(mesh, bones);
    addColor(basemesh, bones);
}


void Skinning::addColor(std::shared_ptr<BaseMesh> basemesh, std::vector<Bone>& bones)
{
    for (auto& bone : bones)
    {
        basemesh->addcolor(bone.color);
    }
}

void Skinning::calculateSkinning(MyMesh& mesh, std::vector<Bone>& bones)
{
    clean(mesh, bones); // TODO: May be to execute()
    for (auto v : mesh.vertices())
    {
        double min_val = std::numeric_limits<double>::infinity();
        Vec actualPoint = Vec(mesh.point(v)[0], mesh.point(v)[1], mesh.point(v)[2]);
        int indexof = 0;
        for (int i = 0; i < bones.size(); i++)
        {
            double bonedistance = std::numeric_limits<double>::infinity(); ;
            for (int j = 0; j < bones[i].points.size(); j++)
            {
                double distancFromBone = distance(bones[i].points[j], actualPoint);
                // closest weight
                if (distancFromBone < min_val)
                {
                    min_val = distancFromBone;
                    indexof = i;
                }
                // closest distanse in the bone
                if (distancFromBone < bonedistance)
                {
                    bonedistance = distancFromBone;
                }
            }
            mesh.data(v).distance[i] = bonedistance;
        }
        mesh.data(v).weigh[indexof] = 1;
        mesh.data(v).idx_of_closest_bone = indexof;
    }
}



Skinning::~Skinning()
{

}


double Skinning::distance(Vec p, Vec p1)
{
    double len = sqrt(pow(p.x - p1.x, 2) + pow(p.y - p1.y, 2) + pow(p.z - p1.z, 2));

    return len;
}