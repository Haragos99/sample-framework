#include "Blendshape.h"


Blendshape::Blendshape(std::shared_ptr<BaseMesh> basemesh)
{
	blendshapemesh = basemesh;
	auto& mesh = basemesh->getMesh();

	for (auto& v : mesh.vertices())
	{
		origanls[v] = mesh.data(v).original;
	}
}




void Blendshape::addNewPostion(std::shared_ptr<BaseMesh> basemesh)
{
	auto& mesh = basemesh->getMesh();
	PointsMap target;
	for (auto& v : mesh.vertices())
	{
		if (mesh.point(v) != mesh.data(v).original)
		{
			target[v] = mesh.point(v);
			modifyVerteces.insert(v);
		}
		
	}
	targets.push_back(std::pair<double,PointsMap>(0.0,target));
}


void Blendshape::setWeight(int i, double weigt)
{
	if (targets[i].first <= 1.0)
	{
		targets[i].first = weigt;
	}
}



MyMesh::Point Blendshape::getBlendMove(MyMesh::VertexHandle v)
{
	MyMesh::Point result(0,0,0);
	for (auto target : targets)
	{
		float weight = target.first;
		PointsMap targetpoints = target.second;
		MyMesh::Point delta = targetpoints[v] - origanls[v];
		result += weight * delta;
	}

	return result;
}

void Blendshape::calculate()
{
	auto& mesh = blendshapemesh->getMesh();

	for (auto& v : modifyVerteces)
	{
		mesh.point(v) = origanls[v] + getBlendMove(v);
	}
}