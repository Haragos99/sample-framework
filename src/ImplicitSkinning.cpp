#include "MyViewer.h"
#include "ImplicitSkinning.h"


void ImplicitSkinning::seperateMesh(std::shared_ptr<BaseMesh> basemesh, int nbones)
{
	MyMesh& mesh = basemesh->getMesh();
	for (int i = 0; i < nbones; i++)
	{
		MyMesh meshpart;
		std::shared_ptr<BaseMesh> basemeshpart;
		for (auto f : mesh.faces()) {
			std::vector<MyMesh::VertexHandle> handles, tri;
			for (auto v : mesh.fv_range(f))
			{
				double weigh = mesh.data(v).weigh[i];
				if (weigh != 0)
				{

					MyMesh::VertexHandle vHandle = meshpart.add_vertex(mesh.point(v));
					handles.push_back(vHandle);
					auto normal = mesh.normal(v);
					meshpart.set_normal(vHandle, normal);

				}
			}
			if (handles.size() != 0)
				meshpart.add_face(handles);
		}
		std::shared_ptr<HRBF> hrbf;
		implicitspaces.push_back(hrbf);
		basemeshpart->setMesh(meshpart);
		separetmeshes.push_back(basemeshpart);
	}
}


void ImplicitSkinning::execute(std::shared_ptr<BaseMesh> basemesh, std::vector<Bone>& bones)
{
	addColor(basemesh, bones);
	seperateMesh(basemesh, bones.size());
	generatesampels();
	for (int i = 0; i < implicitspaces.size(); i++)
	{
		implicitspaces[i]->Calculate(seprateSampels[i], normalsofsampels[i]);
	}

}



void ImplicitSkinning::generatesampels()
{
	for(auto basemesh : separetmeshes)
	{
		std::vector<SamplePoint> basesamples;
		std::vector<MyMesh::Normal> normals;
		float radius = poissongenerator.generateSamples(1000, basemesh->getMesh(), basesamples);
		auto samples = poissongenerator.poissonDisk(radius, basesamples, normals);
		seprateSampels.push_back(samples);
		normalsofsampels.push_back(normals);
	}
}



