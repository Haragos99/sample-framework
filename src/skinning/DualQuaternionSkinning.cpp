#pragma once
#include "DualQuaternionSkinning.h"


/*
 Refact it later
*/

// Maybe try with Bones
void DualQuaternionSkinning::animatemesh(std::shared_ptr<BaseMesh> basemesh, std::vector<Bone>& bones, bool inv)
{
	auto& mesh = basemesh->getMesh();

	for (auto& v : mesh.vertices())
	{
		DualQuaternion dqblend = DualQuaternion(MyQuaternion(0.0f, 0.0f, 0.0f, 0.0f), MyQuaternion(0.0f, 0.0f, 0.0f, 0.0f));
		Mat4 M_result = Mat4(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
		for (int i = 0; i < bones.size(); i++)
		{
			Quaternion q = bones[i].end->quaternion;
			auto w = mesh.data(v).weigh;
			double weight = mesh.data(v).weigh[i];
			DualQuaternion dq = DualQuaternion(MyQuaternion(q[3], q[0], q[1], q[2]), bones[i].end->pivot);
			dqblend = dqblend + (dq * weight);
			Mat4 M = bones[i].end->M.skalar(weight);
			M_result += M;
		}
		Vec point;
		if (!inv)
		{
			point = Vec(mesh.point(v)[0], mesh.point(v)[1], mesh.point(v)[2]);
		}
		else {
			point = Vec(mesh.data(v).original[0], mesh.data(v).original[1], mesh.data(v).original[2]);
		}

		
		Quaternion qa;
		double m[3][3] = {
					{ M_result[0][0], M_result[0][1], M_result[0][2]},
					{ M_result[1][0], M_result[1][1], M_result[1][2] },
					{ M_result[2][0], M_result[2][1], M_result[2][2]}
		};



		//qa.setFromRotationMatrix(m);
		qa = bones[1].end->quaternion;
		Vec s = qa.rotate(point);
		Vec4 point4 = Vec4(mesh.point(v)[0], mesh.point(v)[1], mesh.point(v)[2], 1);
		Vec4 r = point4 * M_result;
		Vec result = dqblend.transform(point);
		OpenMesh::Vec3d newposition = OpenMesh::Vec3d(result.x, result.y, result.z);
		mesh.point(v) = newposition;
		
	}
}

