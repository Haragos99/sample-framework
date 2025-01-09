#include "InverseKinematics.h"


double InverseKinematics::distance(Vec p, Vec p1)
{
	double len = sqrt(pow(p.x - p1.x, 2) + pow(p.y - p1.y, 2) + pow(p.z - p1.z, 2));

	return len;
}


void InverseKinematics::setPosition(Vec& _position)
{
	position = _position;
}

std::vector<Vec>  InverseKinematics::inversekinematics(Joint* joint)
{
	std::vector<Vec> ik;
	joint->to_array(joint, ik);
	double dis = abs(distance(ik[0], position));

	std::vector<double> distances;
	double total_distance = 0.0;
	//ik where we store the old points for the calculation of the ik
	for (int i = 0; i < ik.size() - 1; i++)
	{
		double d = abs(distance(ik[i], ik[i + 1]));
		distances.push_back(d);
		total_distance += d;

	}
	//ik = points;
	if (dis > total_distance)
	{
		for (int i = 0; i < ik.size() - 1; i++)
		{
			double r = abs(distance(position, ik[i]));
			double d = distances[i];
			double alfa = d / r;
			ik[i + 1] = (1 - alfa) * ik[i] + alfa * position;
		}
	}
	else
	{
		Vec b = ik[0];
		double diff = abs(distance(ik.back(), position));
		double tol = 0.001;
		int iter = 0;
		while (diff > tol && iter++ < 100)
		{
			ik.back() = position;
			for (int i = ik.size() - 2; i >= 0; i--)
			{
				double d = distances[i];
				double r = abs(distance(ik[i], ik[i + 1]));
				double alfa = d / r;
				ik[i] = (1 - alfa) * ik[i + 1] + alfa * ik[i];
			}
			ik[0] = b;
			for (int i = 0; i < ik.size() - 2; i++)
			{
				double d = distances[i];
				double r = abs(distance(ik[i], ik[i + 1]));
				double alfa = d / r;
				ik[i + 1] = (1 - alfa) * ik[i] + alfa * ik[i + 1];
			}
			diff = abs(distance(ik.back(), position));
		}
	}
	return ik;
}




void InverseKinematics::execute(int selectedjoint)
{
	Joint* joint = skelton->getSelectedJoint(selectedjoint);
	auto targetpoints =inversekinematics(joint);
	for (auto& p : targetpoints)
	{
		glDisable(GL_LIGHTING);
		glColor3d(0,1,0);
		glPointSize(50.0);
		glBegin(GL_POINTS);
		glVertex3dv(p);
		glEnd();
		glEnable(GL_LIGHTING);
	}

	skelton->calculateMatrix(targetpoints, joint);
	skelton->animateMesh(true);
	

}