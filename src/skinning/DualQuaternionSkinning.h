#pragma once
#include "DualQuaternion.h"
#include "src/skeleton/Joint.h"
#include "src/skeleton/Bone.h"
#include "src/BaseMesh.h"

class DualQuaternionSkinning
{
public:

	void animatemesh(std::shared_ptr<BaseMesh> basemesh, std::vector<Bone>& joints, bool inv);
};