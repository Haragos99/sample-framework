#pragma once
#include <Windows.h>    // Required for NuiApi.h
#include <NuiApi.h>
#include <vector>
#include "Matrix4.h"
#include "skeleton/Skelton.h"
#define IDC_VIDEOVIEW           1003
class KinectSkelton {
	// Current Kinect
	INuiSensor* NuiSensor;
	HANDLE NextSkeletonEvent;
	HANDLE SkeletonStreamHandle;
	HWND  m_hWnd;
	std::vector<Bone> bones;
	std::vector<Vec> joints;
	


public:
	void setSkeltonTracking() { if (NuiSensor) { NuiSensor->NuiSkeletonTrackingEnable(NextSkeletonEvent, 0); } }

	void genareteSkelton();

	void ProcessSkeleton();

	void DrawBone(const NUI_SKELETON_DATA& skel, NUI_SKELETON_POSITION_INDEX joint0, NUI_SKELETON_POSITION_INDEX joint1,int boneindex);
	
	Skelton skelton;

	HRESULT CreateFirstConnected();

	void creatSkelton(NUI_SKELETON_DATA& skel);

	void update();

	KinectSkelton();

	void draw(Vis::Visualization& vis);

	~KinectSkelton();
	
};
