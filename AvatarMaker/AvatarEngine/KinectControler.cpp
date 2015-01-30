//Implementation of Kinect Driver

#include "stdafx.h"
#include <stdio.h>
#include "KinectControler.h"
#include "NuiUtility.h"

using namespace std;
// Static initializers
LPCWSTR KinectControler::GrammarFileName = L"data\\BodyAvatarCommandGrammar.grxml";

// This is the class ID we expect for the Microsoft Speech recognizer.
// Other values indicate that we're using a version of sapi.h that is
// incompatible with this sample.
DEFINE_GUID(CLSID_ExpectedRecognizer, 0x495648e7, 0xf7ab, 0x4267, 0x8e, 0x0f, 0xca, 0xfb, 0x7a, 0x33, 0xc1, 0x60);

static const COLORREF g_JointColorTable[NUI_SKELETON_POSITION_COUNT] = 
{
	RGB(169, 176, 155), // NUI_SKELETON_POSITION_HIP_CENTER
	RGB(169, 176, 155), // NUI_SKELETON_POSITION_SPINE
	RGB(168, 230, 29),  // NUI_SKELETON_POSITION_SHOULDER_CENTER
	RGB(200, 0,   0),   // NUI_SKELETON_POSITION_HEAD
	RGB(79,  84,  33),  // NUI_SKELETON_POSITION_SHOULDER_LEFT
	RGB(84,  33,  42),  // NUI_SKELETON_POSITION_ELBOW_LEFT
	RGB(255, 126, 0),   // NUI_SKELETON_POSITION_WRIST_LEFT
	RGB(215,  86, 0),   // NUI_SKELETON_POSITION_HAND_LEFT
	RGB(33,  79,  84),  // NUI_SKELETON_POSITION_SHOULDER_RIGHT
	RGB(33,  33,  84),  // NUI_SKELETON_POSITION_ELBOW_RIGHT
	RGB(77,  109, 243), // NUI_SKELETON_POSITION_WRIST_RIGHT
	RGB(37,   69, 243), // NUI_SKELETON_POSITION_HAND_RIGHT
	RGB(77,  109, 243), // NUI_SKELETON_POSITION_HIP_LEFT
	RGB(69,  33,  84),  // NUI_SKELETON_POSITION_KNEE_LEFT
	RGB(229, 170, 122), // NUI_SKELETON_POSITION_ANKLE_LEFT
	RGB(255, 126, 0),   // NUI_SKELETON_POSITION_FOOT_LEFT
	RGB(181, 165, 213), // NUI_SKELETON_POSITION_HIP_RIGHT
	RGB(71, 222,  76),  // NUI_SKELETON_POSITION_KNEE_RIGHT
	RGB(245, 228, 156), // NUI_SKELETON_POSITION_ANKLE_RIGHT
	RGB(77,  109, 243)  // NUI_SKELETON_POSITION_FOOT_RIGHT
};

static const COLORREF g_SkeletonColors[NUI_SKELETON_COUNT] =
{
	RGB( 255, 0, 0),
	RGB( 0, 255, 0 ),
	RGB( 64, 255, 255 ),
	RGB( 255, 255, 64 ),
	RGB( 255, 64, 255 ),
	RGB( 128, 128, 255 )
};

//joint read array
/*
NUI_SKELETON_POSITION_HIP_CENTER	= 0,
NUI_SKELETON_POSITION_SPINE	= 1
NUI_SKELETON_POSITION_SHOULDER_CENTER	= 2
NUI_SKELETON_POSITION_HEAD	= 3
NUI_SKELETON_POSITION_SHOULDER_LEFT	= 4
NUI_SKELETON_POSITION_ELBOW_LEFT	= 5
NUI_SKELETON_POSITION_WRIST_LEFT	= 6
NUI_SKELETON_POSITION_HAND_LEFT	= 7
NUI_SKELETON_POSITION_SHOULDER_RIGHT	= 8
NUI_SKELETON_POSITION_ELBOW_RIGHT	= 9
NUI_SKELETON_POSITION_WRIST_RIGHT	= 10
NUI_SKELETON_POSITION_HAND_RIGHT	= 11
NUI_SKELETON_POSITION_HIP_LEFT	= 12
NUI_SKELETON_POSITION_KNEE_LEFT	= 13
NUI_SKELETON_POSITION_ANKLE_LEFT	= 14
NUI_SKELETON_POSITION_FOOT_LEFT	= 15
NUI_SKELETON_POSITION_HIP_RIGHT	= 16
NUI_SKELETON_POSITION_KNEE_RIGHT	= 17
NUI_SKELETON_POSITION_ANKLE_RIGHT	= 18
NUI_SKELETON_POSITION_FOOT_RIGHT	= 19
NUI_SKELETON_POSITION_COUNT	= 20*/

//lookups for color tinting based on player index
static const int g_IntensityShiftByPlayerR[] = { 1, 2, 0, 2, 0, 0, 2, 0 };
static const int g_IntensityShiftByPlayerG[] = { 1, 2, 2, 0, 2, 0, 0, 1 };
static const int g_IntensityShiftByPlayerB[] = { 1, 0, 2, 2, 0, 2, 0, 2 };


//-------------------------------------------------------------------
// Public Methods for reading Data from KINECT
//-------------------------------------------------------------------
SpeechCommands KinectControler::GetSpeechCommand()
{
	return m_pCommandStream->GetStreamFrame();
}

NUI_SKELETON_FRAME* KinectControler::GetSkeletonFrame()
{
	return m_pSkeletonStreamBuffer->GetLatestSkeleton();
//	return m_pSkeletonStreamBuffer->GetSkeleton();
}

KinectControler::KinectControler()
{
	m_pNuiSensor = nullptr;
	HRESULT hr = InitializeNuiSensor();
	if (FAILED(hr))
		throw new std::exception("KINECT Controler Initialize failed.");
	//hr = InitializeSkeletonStream();
	m_pSkeletonStreamBuffer.reset( new SkeletonStreamBufferViewer);
	m_pSkeletonStream.reset( new Nui::NuiSkeletonStream(m_pNuiSensor));
	m_pSkeletonStream->SetStreamViewer(m_pSkeletonStreamBuffer.get());
	m_pCommandStream.reset( new SpeechCommandStream(m_pNuiSensor,GrammarFileName));
}

KinectControler::~KinectControler()
{
	Release();
}

//-------------------------------------------------------------------
// Nui_Init
//
// Initialize Kinect by instance name
//-------------------------------------------------------------------
HRESULT KinectControler::InitializeNuiSensor( OLECHAR *instanceName )
{
	// Generic creation failure
	if ( NULL == instanceName )
	{
		return E_FAIL;
	}

	HRESULT hr = NuiCreateSensorById( instanceName, &m_pNuiSensor );
	
	// Generic creation failure
	if ( FAILED(hr) )
	{
		return hr;
	}

	return InitializeNuiSensor();
}

//-------------------------------------------------------------------
// Nui_Init
//
// Initialize Kinect
//-------------------------------------------------------------------

INuiSensor* GetFirstReadySensor()
{
	int iCount = 0;
	HRESULT hr = NuiGetSensorCount(&iCount);
	if (FAILED(hr))
	{
		return nullptr;
	}

	for (int i = 0; i < iCount; ++i)
	{
		INuiSensor* pNuiSensor = nullptr;

		if (SUCCEEDED(NuiCreateSensorByIndex(i, &pNuiSensor)))
		{
			if (SUCCEEDED(pNuiSensor->NuiStatus()))
				return pNuiSensor;
		}
		SafeRelease(pNuiSensor);
	}
	return nullptr;
}

HRESULT KinectControler::InitializeNuiSensor()
{
	HRESULT  hr;
	if (m_pNuiSensor == nullptr)
		m_pNuiSensor = GetFirstReadySensor();
	DWORD nuiFlags = NUI_INITIALIZE_FLAG_USES_SKELETON | NUI_INITIALIZE_FLAG_USES_AUDIO;

	if (!m_pNuiSensor) 
	{
		printf("Can't find any ready NUI Sensor.\n");
		return ERROR_DEVICE_NOT_AVAILABLE;
	}

	hr = m_pNuiSensor->NuiInitialize( nuiFlags );
  
	if ( FAILED( hr ) )
	{
		if ( E_NUI_SKELETAL_ENGINE_BUSY == hr )
		{
			printf("KINECT Skeleton Engine is busy.\n");
		}
		if ( E_NUI_DEVICE_IN_USE == hr )
		{
			printf("USB Link Error in Initialize KINECT.\n");
		}
		else
		{
			printf("Error in Initialize NUI API.\n");
		}
	}
	return hr;
}

HRESULT KinectControler::StartStream()
{
	m_pSkeletonStream->SetChooserMode(Nui::ChooserMode::ChooserModeSticky2);
	m_ThreadTerminatingSignal = false;
	m_pStreamingThread.reset(new std::thread([&]() {
		while (!m_ThreadTerminatingSignal)
		{
			m_pSkeletonStream->ProcessStreamFrame();
		}
	}));

	cout<<"Thread 0x"<<hex<<std::this_thread::get_id()<<" is initializing speech stream..."<<endl;
	HRESULT hr = m_pCommandStream->StartStream();
	if (FAILED(hr))
	{
		cout<<"Fail in start speech command stream , error code = 0x"<<hex<<hr<<endl;
		return hr;
	}

	return S_OK;
}

//-------------------------------------------------------------------
// Nui_UnInit
//
// Uninitialize Kinect
//-------------------------------------------------------------------
void KinectControler::Release()
{
	m_ThreadTerminatingSignal = true;
	if (m_pStreamingThread->joinable() )

		m_pStreamingThread->join();

	if ( m_pNuiSensor )
	{
		m_pNuiSensor->NuiShutdown();
		m_pNuiSensor->Release();
		m_pNuiSensor = NULL;
	}

}
