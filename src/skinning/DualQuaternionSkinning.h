#pragma once
#include "DualQuaternion.h"
#include "../skeleton/Joint.h"
#include "../skeleton/Bone.h"
#include "../BaseMesh.h"

class DualQuaternionSkinning
{
public:

	void animatemesh(std::shared_ptr<BaseMesh> basemesh, std::vector<Bone>& joints, bool inv);
};