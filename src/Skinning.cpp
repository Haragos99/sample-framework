#include "Skinning.h"


Skinning::Skinning(Skelton& skelton_, MyMesh& mesh_): skelton(skelton_),mesh(mesh_){}



void Skinning::clean(MyMesh::VertexHandle& v)
{
    mesh.data(v).weigh.clear();
    mesh.data(v).weigh.resize(skelton.getSize(), 0.0);
    mesh.data(v).distance.clear();
    mesh.data(v).distance.resize(skelton.getSize(), std::numeric_limits<double>::infinity());
}

void Skinning::calculateSkinning()
{
    for (auto v : mesh.vertices())
    {
        clean(v);
        double min_val = std::numeric_limits<double>::infinity();
        Vec actualPoint = Vec(mesh.point(v)[0], mesh.point(v)[1], mesh.point(v)[2]);
        int indexof = 0;
        for (int i = 0; i < skelton.getSize(); i++)
        {
            double closestBone = std::numeric_limits<double>::infinity(); ;
            for (int j = 0; j < skelton.bones[i].points.size(); j++)
            {
                double distancFromBone = distance(skelton.bones[i].points[j], actualPoint);
                // closest weight
                if (distancFromBone < min_val)
                {
                    min_val = distancFromBone;
                    indexof = i;
                }
                // closest distanse in the bone
                if (distancFromBone < closestBone)
                {
                    closestBone = distancFromBone;
                }
            }
            mesh.data(v).distance[i] = closestBone;
        }
        mesh.data(v).weigh[indexof] = 1;
        mesh.data(v).idx_of_closest_bone = indexof;
    }
}



double Skinning::distance(Vec p, Vec p1)
{
    double len = sqrt(pow(p.x - p1.x, 2) + pow(p.y - p1.y, 2) + pow(p.z - p1.z, 2));

    return len;
}