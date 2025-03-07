#include "Sculpt.h"

void Sculpt::grab(int selected, const Vec& position)
{
	auto& mesh = basemesh->getMesh();
	MyMesh::VertexHandle selectedHandle(selected);
	if (!mesh.is_valid_handle(selectedHandle))
	{
		return;
	}
	
	Vec actualpoint = Vec(mesh.point(selectedHandle).data());
	Vec diff = position - actualpoint;
	for (auto v : mesh.vertices())
	{
		Vec point = Vec(mesh.point(v).data());
		double dis = distance(actualpoint, point);
		if (radius > dis)
		{
			double F = 1 - ((dis / radius));
			Vec newdiff = diff * F;
			mesh.point(v) += MyMesh::Point(newdiff.x, newdiff.y, newdiff.z);
		}
	}
}

void Sculpt::smooth(int selected, const Vec& position)
{
	auto& mesh = basemesh->getMesh();
	MyMesh::VertexHandle selectedHandle(selected);
	if (!mesh.is_valid_handle(selectedHandle))
	{
		return;
	}
	Vec actualpoint = Vec(mesh.point(selectedHandle).data());
	double smootingfactor = 0.5;
	for (int i = 0; i < 10; i++)
	{
		auto smooth = mesh;
		for (auto v : mesh.vertices()) {
			Vec point = Vec(mesh.point(v).data());
			double dis = distance(actualpoint, point);
			if (radius > dis)
			{
				Vec Avg;
				int n = 0;
				for (auto vi : mesh.vv_range(v)) {
					Vec vertex = Vec(mesh.point(vi));
					Avg += vertex;
					n++;
				}
				Avg /= n;
				MyMesh::Point pointavg = MyMesh::Point(Avg.x, Avg.y, Avg.z);
				smooth.point(v) += smootingfactor * (pointavg - mesh.point(v));
			}

		}

		for (auto v : smooth.vertices()) {
			mesh.point(v) = smooth.point(v);
		}
	}
}

double Sculpt::distance(Vec p, Vec p1)
{
	double len = sqrt(pow(p.x - p1.x, 2) + pow(p.y - p1.y, 2) + pow(p.z - p1.z, 2));
	return len;
}

void Sculpt::excecut(int selected, const Vec& position)
{
	switch (selectedType)
	{
	case Vis::SculptType::GRAB:
		grab(selected, position);
		break;
	case Vis::SculptType::SMOOTH:
		smooth(selected, position);
		break;
	default:
		break;
	}
}