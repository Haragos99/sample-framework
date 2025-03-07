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
		target[v] = mesh.point(v);
	}
	targets.push_back(std::pair<int,PointsMap>(1.0,target));
}


void Blendshape::setWeight(int i, double weigt)
{
	targets[i].first = weigt;
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

	for (auto& v : mesh.vertices())
	{
		mesh.point(v) = origanls[v] + getBlendMove(v);
	}
}