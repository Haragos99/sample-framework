#pragma once
#include "DualQuaternion.h"
#include "src/skeleton/Joint.h"
#include "src/BaseMesh.h"

class DualQuaternionSkinning
{
public:

	void animatemesh(std::shared_ptr<BaseMesh> basemesh, std::vector<Joint*>& joints, bool inv);
};