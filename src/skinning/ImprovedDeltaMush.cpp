#include "ImprovedDeltaMush.h"
#include <QtWidgets>



void ImprovedDeltaMush::execute(std::shared_ptr<BaseMesh> basemesh, std::vector<Bone>& bones)
{
	DeltaMush::execute(basemesh, bones);
	SetDistance(basemesh, bones);
}
void ImprovedDeltaMush::animatemesh(std::shared_ptr<BaseMesh> basemesh, std::vector<Bone>& bones, bool inv)
{
	Skinning::animatemesh(basemesh, bones, inv);
	executeCCD(basemesh);
}


void ImprovedDeltaMush::projectPointToPlane(const MyMesh::Point& P, const MyMesh::Normal& N, MyMesh::Point& Q) {
	// normalize the normal
	MyMesh::Normal norm = N;
	norm.normalize();

	// distance between the pointand the plane
	float d = dot(P - Q, norm);

	//  projected point
	Q = Q - (d * norm);
}


void ImprovedDeltaMush::executeCCD(std::shared_ptr<BaseMesh> basemesh)
{
	Delta_Mush_two(basemesh);
	Collison collison;

	auto smooth_basemesh = smoothMesh(basemesh);
	MyMesh& mesh = basemesh->getMesh();
	MyMesh& smooth_mesh = smooth_basemesh->getMesh();

	collison.init(delta);
	collison.colliedfaces = colliedfaces;
	collison.colliedverteces = colliedverteces;
	collison.colliededges = colliededges;
	emit startProgress(tr("Improve"));
	while (collison.collisondetec(mesh, smooth_mesh))
	{
		Delta_Mush_two(basemesh);
		float alfa = collison.getAlfa();
		int percent = alfa * 100;
		collison.setAlfa(0);
		emit progressUpdated(percent);
	}
	emit endProgress();
	
	//smoothcollison(collison.verteces, basemesh->getMesh()); //TODO: Refactor
}


void ImprovedDeltaMush::smoothcollison(std::set<MyMesh::VertexHandle> verteces, MyMesh& mesh)
{
	float smootingfactor = 0.4;
	for (int i = 0; i < 2; i++)
	{
		for (auto v : verteces)
		{
			Vec Avg;
			int n = 0;
			for (auto vi : mesh.vv_range(v)) {

				auto point = mesh.point(vi);
				projectPointToPlane(mesh.point(v), mesh.normal(v), point);
				Vec vertex = Vec(point);
				Avg += vertex;
				n++;
			}
			Avg /= n;
			MyMesh::Point pointavg = MyMesh::Point(Avg.x, Avg.y, Avg.z);
			mesh.point(v) += smootingfactor * (pointavg - mesh.point(v));
		}
	}
}

//TODO: Must Refact this 
void ImprovedDeltaMush::SetDistance(std::shared_ptr<BaseMesh> basemesh, std::vector<Bone>& bones)
{
	int index =0;
	MyMesh& mesh = basemesh->getMesh();
	float factor = 1.75;
	colliedverteces.clear();
	colliedfaces.clear();
	colliededges.clear();
	int fn = mesh.n_faces();
	int en = mesh.n_edges();
	int nv = mesh.n_vertices();
	for (auto v : mesh.vertices())
	{

		Vec meshpoint = Vec(mesh.point(v));
		for (auto b : bones)
		{
			if (!b.isLastBone())
			{
				Vec bonepoint = bones[index].end->point;
				float distance = (meshpoint - bonepoint).norm();

				if (distance <= bones[index].lenght() / factor)
				{

					mesh.data(v).color = Vec(0, 1, 0);
					colliedverteces.emplace(v);
				}
			}
			break;

		}

	}
	for (auto f : mesh.faces())
	{
		MyMesh::Point centroid;
		int vertexcount = 0;
		for (auto v : mesh.fv_range(f))
		{
			centroid += mesh.point(v);
			vertexcount++;
		}
		centroid /= static_cast<float>(vertexcount);
		for (auto b : bones)
		{
			if (!b.isLastBone())
			{
				Vec bonepoint = bones[index].end->point;
				float distance = (Vec(centroid) - bonepoint).norm();
				if (distance <= bones[index].lenght() / factor)
				{
					colliedfaces.emplace(f);
				}

			}
			break;
		}

	}
	for (auto e : mesh.edges())
	{
		MyMesh::EdgeHandle eh1 = e;

		// Get the start and end vertices of the first edge
		MyMesh::HalfedgeHandle heh1 = mesh.halfedge_handle(eh1, 0);  // First halfedge
		MyMesh::VertexHandle v0_1 = mesh.from_vertex_handle(heh1);   // Start vertex of edge 1
		MyMesh::VertexHandle v1_1 = mesh.to_vertex_handle(heh1);     // End vertex of edge 1
		Vec edge1 = Vec(mesh.point(v0_1));
		Vec edge2 = Vec(mesh.point(v1_1));

		for (auto b : bones)
		{
			if (!b.isLastBone())
			{
				Vec bonepoint = bones[index].end->point;
				float distance1 = (edge1 - bonepoint).norm();
				float distance2 = (edge2 - bonepoint).norm();

				if (distance1 <= bones[index].lenght() / factor || distance2 <= bones[index].lenght() / factor)
				{
					colliededges.emplace(e);
				}

			}
			break;
		}


	}



}


