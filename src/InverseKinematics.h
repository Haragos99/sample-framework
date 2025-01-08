#pragma once
#include "Skelton.h"


class InverseKinematics {
public:

	InverseKinematics() = default;
	InverseKinematics(std::shared_ptr<Skelton> _skelton, Vec _position) : skelton(_skelton) , position(_position){}
	void execute(int selectedjoint);
	void setPosition(Vec& _position);
	~InverseKinematics() = default;


private:
	double distance(Vec p, Vec p1);
	std::vector<Vec> inversekinematics(Joint* joint);
	std::shared_ptr<Skelton> skelton;
	Vec position;


};