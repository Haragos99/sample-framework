#pragma once
#include "DualQuaternionSkinning.h"

void DualQuaternionSkinning::animatemesh(std::shared_ptr<BaseMesh> basemesh, std::vector<Joint*>& joints, bool inv)
{
	auto& mesh = basemesh->getMesh();

	for (auto& v : mesh.vertices())
	{
		DualQuaternion dqblend = DualQuaternion(Quaternion(0.0f, 0.0f, 0.0f, 0.0f), Quaternion(0.0f, 0.0f, 0.0f, 0.0f));

		for (int i = 0; i < joints.size(); i++)
		{
			double weight = mesh.data(v).weigh[i];
			DualQuaternion dq = DualQuaternion(joints[i]->M);
			dqblend = dqblend + dq * weight;
		}
		Vec point;
		if (!inv)
		{
			point = Vec(mesh.point(v)[0], mesh.point(v)[1], mesh.point(v)[2]);
		}
		else {
			point = Vec(mesh.data(v).original[0], mesh.data(v).original[1], mesh.data(v).original[2]);
		}


		Vec result = dqblend.transform(point);
		OpenMesh::Vec3d newposition = OpenMesh::Vec3d(result.x, result.y, result.z);
		mesh.point(v) = newposition;
	}
}

