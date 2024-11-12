#pragma once
#include <Windows.h>    // Required for NuiApi.h
#include <NuiApi.h>
#include <vector>
#include "src/Matrix4.h"
#include "src/Bone.h"
#define IDC_VIDEOVIEW           1003
class KinectSkelton {
	// Current Kinect
	INuiSensor* m_pNuiSensor;
	HANDLE m_hNextSkeletonEvent;
	HANDLE m_pSkeletonStreamHandle;
	HWND  m_hWnd;
	std::vector<Bone> bones;
	std::vector<Joint*> joints;
	


public:
	void setSkeltonTracking(){ m_pNuiSensor->NuiSkeletonTrackingEnable(m_hNextSkeletonEvent, 0); }

	void genareteSkelton();

	void ProcessSkeleton();

	void DrawBone(const NUI_SKELETON_DATA& skel, NUI_SKELETON_POSITION_INDEX joint0, NUI_SKELETON_POSITION_INDEX joint1,int boneindex);
    
	Skelton skelton;

	HRESULT CreateFirstConnected();

	void creatSkelton(NUI_SKELETON_DATA& skel);

	void update();

	KinectSkelton();

	void draw();

	~KinectSkelton();
	
};
