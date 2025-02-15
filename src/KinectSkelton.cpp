#include "KinectSkelton.h"


KinectSkelton::KinectSkelton(): NextSkeletonEvent(INVALID_HANDLE_VALUE), SkeletonStreamHandle(INVALID_HANDLE_VALUE) , NuiSensor(nullptr)
{

    joints.resize(20);
    skelton.loadFile("C:\\Dev\\sample-framework-master\\Bones\\tpose.bone");
    skelton.build();

}

void KinectSkelton::genareteSkelton()
{



    




}


HRESULT KinectSkelton::CreateFirstConnected()
{
    INuiSensor* pNuiSensor;

    int iSensorCount = 0;
    HRESULT hr = NuiGetSensorCount(&iSensorCount);
    if (FAILED(hr))
    {
        return hr;
    }

    // Look at each Kinect sensor
    for (int i = 0; i < iSensorCount; ++i)
    {
        // Create the sensor so we can check status, if we can't create it, move on to the next
        hr = NuiCreateSensorByIndex(i, &pNuiSensor);
        if (FAILED(hr))
        {
            continue;
        }
        
        // Get the status of the sensor, and if connected, then we can initialize it
        hr = pNuiSensor->NuiStatus();
        if (S_OK == hr)
        {
            NuiSensor = pNuiSensor;
            break;
        }

        // This sensor wasn't OK, so release it since we're not using it
        pNuiSensor->Release();
    }

    if (NULL != NuiSensor)
    {
        // Initialize the Kinect and specify that we'll be using skeleton
        hr = NuiSensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_SKELETON);
        if (SUCCEEDED(hr))
        {
            // Create an event that will be signaled when skeleton data is available
            NextSkeletonEvent = CreateEventW(NULL, TRUE, FALSE, NULL);

            // Open a skeleton stream to receive skeleton data
            hr = NuiSensor->NuiSkeletonTrackingEnable(NextSkeletonEvent, 0);
        }
    }

    if (NULL == NuiSensor || FAILED(hr))
    {
        return E_FAIL;
    }

    return hr;
}


void KinectSkelton::update()
{
    if (NULL == NuiSensor)
    {
        return;
    }

    // Wait for 0ms, just quickly test if it is time to process a skeleton
    if (WAIT_OBJECT_0 == WaitForSingleObject(NextSkeletonEvent, 0))
    {
        ProcessSkeleton();
    }
}



void KinectSkelton::creatSkelton(NUI_SKELETON_DATA& skel)
{
    
    NUI_SKELETON_BONE_ORIENTATION boneOrientations[NUI_SKELETON_POSITION_COUNT];
    HRESULT hr = NuiSkeletonCalculateBoneOrientations(&skel, boneOrientations);
    if (FAILED(hr))
    {
        return;
    }
    for (int i = 0; i < NUI_SKELETON_POSITION_COUNT; ++i)
    {

        Vec point = Vec(skel.SkeletonPositions[i].x / skel.SkeletonPositions[i].w, skel.SkeletonPositions[i].y / skel.SkeletonPositions[i].w, skel.SkeletonPositions[i].z / skel.SkeletonPositions[i].w);
        //joints[i] = new Joint(poin, i);
        Joint* joint = skelton.root->searchbyid(skelton.root, i);
        joints[i] = point;
        //joints[i] = point;
        const NUI_SKELETON_BONE_ORIENTATION& boneOrientation = boneOrientations[i];
        Matrix4 m = boneOrientation.hierarchicalRotation.rotationMatrix;



        auto q = boneOrientation.absoluteRotation.rotationQuaternion;
        qglviewer::Quaternion quaternion(q.x, q.y, q.z, q.w);
        Mat4 matrix(m.M11, m.M12, m.M13, m.M14, 
                    m.M21, m.M22, m.M23, m.M24,
                    m.M31, m.M32, m.M33, m.M34,
                    m.M41, m.M42, m.M43, m.M44);
        Vec axies;
        qreal angel;


        quaternion.getAxisAngle(axies, angel);

        joint->M = matrix;

        joint->R = matrix;
        Vec pivot;
        if (joint->parent!= nullptr)
        {
            pivot = joint->parent->Tpose;
        }
        else
        {
            pivot = joint->Tpose;
        }

        Mat4 T1 = TranslateMatrix(-pivot);
        Mat4 T2 = TranslateMatrix(pivot);
        Mat4 M = T1 * matrix * T2;
        Vec4 p = Vec4(joint->Tpose) * M;
        //joint->point = Vec(p.x, p.y, p.z);
        joint->point = Vec(p.x,p.y,p.z);
    }
}



void KinectSkelton::ProcessSkeleton()
{
    //m_pNuiSensor->NuiSkeletonTrackingEnable(m_hNextSkeletonEvent,  NUI_SKELETON_TRACKING_FLAG_ENABLE_SEATED_SUPPORT );
    NUI_SKELETON_FRAME skeletonFrame = { 0 };

    HRESULT hr = NuiSensor->NuiSkeletonGetNextFrame(0, &skeletonFrame);
    if (FAILED(hr))
    {
        return;
    }

    // smooth out the skeleton data
    NuiSensor->NuiTransformSmooth(&skeletonFrame, NULL);



    RECT rct;
    GetClientRect(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), &rct);
    int width = rct.right;
    int height = rct.bottom;
    

    for (int i = 0; i < NUI_SKELETON_COUNT; ++i)
    {
        auto trackingState = skeletonFrame.SkeletonData[i].eTrackingState;

        if (NUI_SKELETON_TRACKED == trackingState)
        {
            // We're tracking the skeleton, draw it
            creatSkelton(skeletonFrame.SkeletonData[i]);         
        }

    }

 

}

KinectSkelton::~KinectSkelton()
{
    if (NuiSensor)
    {
        NuiSensor->NuiShutdown();
    }

    if (NextSkeletonEvent && (NextSkeletonEvent != INVALID_HANDLE_VALUE))
    {
        CloseHandle(NextSkeletonEvent);
    }
}

void KinectSkelton::draw(Vis::Visualization& vis)
{
    skelton.draw(vis);
   
    for (auto j : joints)
    {
        
        glColor3d(0.0, 1.0, 0.0);
        glPointSize(50.0);
        glBegin(GL_POINTS);
        glVertex3dv(j);
        glEnd();
        
    }
}



void KinectSkelton::DrawBone(const NUI_SKELETON_DATA& skel, NUI_SKELETON_POSITION_INDEX joint0, NUI_SKELETON_POSITION_INDEX joint1, int boneindex)
{
    NUI_SKELETON_POSITION_TRACKING_STATE joint0State = skel.eSkeletonPositionTrackingState[joint0];
    NUI_SKELETON_POSITION_TRACKING_STATE joint1State = skel.eSkeletonPositionTrackingState[joint1];
    
    // If we can't find either of these joints, exit
    if (joint0State == NUI_SKELETON_POSITION_NOT_TRACKED || joint1State == NUI_SKELETON_POSITION_NOT_TRACKED)
    {
        return;
    }

    // Don't draw if both points are inferred
    if (joint0State == NUI_SKELETON_POSITION_INFERRED && joint1State == NUI_SKELETON_POSITION_INFERRED)
    {
        return;
    }

    // We assume all drawn bones are inferred unless BOTH joints are tracked
    if (joint0State == NUI_SKELETON_POSITION_TRACKED && joint1State == NUI_SKELETON_POSITION_TRACKED)
    {
        //Bone b = Bone(joints[joint0], joints[joint1], boneindex, Vec(0, 1, 0));
        //bones[boneindex] = b;
    }
}