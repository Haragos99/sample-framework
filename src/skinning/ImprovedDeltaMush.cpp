#include "ImprovedDeltaMush.h"
#include <QtWidgets>
#include <igl/Timer.h>


void ImprovedDeltaMush::execute(std::shared_ptr<BaseMesh> basemesh, std::vector<Bone>& bones)
{
	DeltaMush::execute(basemesh, bones);
	SetDistance(basemesh, bones);
}
void ImprovedDeltaMush::animatemesh(std::shared_ptr<BaseMesh> basemesh, std::vector<Bone>& bones, bool inv)
{
	Skinning::animatemesh(basemesh, bones, inv);
	meshm = basemesh->getMesh();
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
	igl::Timer timer;
	timer.start();
	DeltaMush2(delta,basemesh);
	basemesh->setMesh(meshm);
	Collison collison;
	collison.init(delta);
	collison.colliedfaces = colliedfaces;
	collison.colliedverteces = colliedverteces;
	collison.colliededges = colliededges;
	auto sb = smoothMesh(basemesh);
	auto m = sb->getMesh();

	emit startProgress(tr("Improve"));
	while (collison.collisondetec(meshm,smooth ))
	{
		DeltaMush2(delta, basemesh);
		//basemesh->setMesh(meshm); //TODO CHECK THIS  IT WORKS
		float alfa = collison.getAlfa();
		int percent = alfa * 100;
		emit progressUpdated(percent);
		collison.setAlfa(0);		
	}
	timer.stop();
	emit endProgress();
	double tt = timer.getElapsedTimeInSec(); 

	emit displayMessage(std::to_string(tt).c_str());
	basemesh->setMesh(meshm);
	smoothcollison(collison.verteces, basemesh->getMesh()); //TODO: Refactor
}



// TODO: Rethink 
void ImprovedDeltaMush::DeltaMush2(std::vector<Eigen::Vector4d> v, std::shared_ptr<BaseMesh> basemesh)
{
	for (int i = 0; i < v.size(); i++)
	{
		v[i][0] *= deltaMushFactor;
		v[i][1] *= deltaMushFactor;
		v[i][2] *= deltaMushFactor;
	}
	auto m = meshm;
	auto smooth_Basemesh = smoothMesh(basemesh);
	auto smooth_mesh = smooth_Basemesh->getMesh();
	smooth_mesh.request_face_normals();
	smooth_mesh.request_vertex_normals();
	smooth_mesh.update_normals();
	for (auto ve : m.vertices()) {
		Eigen::MatrixXd C(4, 4);
		MyMesh::Normal normal = smooth_mesh.normal(ve);

		MyMesh::HalfedgeHandle heh = *smooth_mesh.voh_iter(ve);
		auto ed = smooth_mesh.calc_edge_vector(heh);
		Vec t = Vec((ed - (ed | normal) * normal).normalize());
		Vec b = (t ^ Vec(normal)).unit();

		C(0, 0) = t[0];
		C(0, 1) = t[1];
		C(0, 2) = t[2];
		C(0, 3) = 0;

		C(1, 0) = normal[0];
		C(1, 1) = normal[1];
		C(1, 2) = normal[2];
		C(1, 3) = 0;

		C(2, 0) = b[0];
		C(2, 1) = b[1];
		C(2, 2) = b[2];
		C(2, 3) = 0;

		C(3, 0) = smooth_mesh.point(ve)[0];
		C(3, 1) = smooth_mesh.point(ve)[1];
		C(3, 2) = smooth_mesh.point(ve)[2];
		C(3, 3) = 1;

		meshm.data(ve).C = C.transpose();

		auto d = C.transpose() * v[ve.idx()];

		meshm.point(ve) = MyMesh::Point(d[0], d[1], d[2]);


	}
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
	int index =9;
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


