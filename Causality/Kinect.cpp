#include <Kinect.h>
#include "Kinect.h"
using namespace Platform::Devices;
using namespace Microsoft::WRL;

JointType Kinect::JointsParent[JointType_Count] = {
	JointType_SpineBase, // JointType_SpineBase, Root
	JointType_SpineBase, // JointType_SpineMid
	JointType_SpineShoulder, // JointType_Neck
	JointType_Neck,  //JointType_Head
	JointType_SpineShoulder,  //JointType_ShoulderLeft
	JointType_ShoulderLeft, // JointType_ElbowLeft
	JointType_ElbowLeft, // JointType_WristLeft
	JointType_WristLeft,  // JointType_HandLeft
	JointType_SpineShoulder,  // JointType_ShoulderRight
	JointType_ShoulderRight,  // JointType_ElbowRight
	JointType_ElbowRight,  // JointType_WristRight
	JointType_WristRight,  // JointType_HandRight
	JointType_SpineBase,  // JointType_HipLeft
	JointType_HipLeft,  // JointType_KneeLeft
	JointType_KneeLeft,  // JointType_AnkleLeft
	JointType_AnkleLeft,  // JointType_FootLeft
	JointType_SpineBase,  // JointType_HipRight
	JointType_HipRight,  // JointType_KneeRight
	JointType_KneeRight,  // JointType_AnkleRight
	JointType_AnkleRight,  // JointType_FootRight
	JointType_SpineMid,  // JointType_SpineShoulder
	JointType_HandLeft, // JointType_HandTipLeft
	JointType_HandLeft,  // JointType_ThumbLeft
	JointType_HandRight,  // JointType_HandTipRight
	JointType_HandRight,  //JointType_ThumbRight
};

class Kinect::Impl
{
	bool IsActive() const
	{
		return m_pKinectSensor;
	}

	HRESULT Initalize()
	{
		using namespace Microsoft::WRL;
		using namespace std;
		HRESULT hr;
		// Current Kinect
		ComPtr<IKinectSensor>          pKinectSensor;
		ComPtr<ICoordinateMapper>      pCoordinateMapper;
		ComPtr<IBodyFrameReader>       pBodyFrameReader;
		ComPtr<IBodyFrameSource>	   pBodyFrameSource = NULL;

		hr = GetDefaultKinectSensor(&pKinectSensor);
		if (FAILED(hr))
		{
			return hr;
		}

		if (m_pKinectSensor)
		{
			// Initialize the Kinect and get coordinate mapper and the body reader

			hr = m_pKinectSensor->Open();

			if (SUCCEEDED(hr))
			{
				hr = m_pKinectSensor->get_CoordinateMapper(&pCoordinateMapper);
			}

			if (SUCCEEDED(hr))
			{
				hr = m_pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);
			}

			if (SUCCEEDED(hr))
			{
				hr = pBodyFrameSource->OpenReader(&pBodyFrameReader);
			}
		}

		if (!m_pKinectSensor || FAILED(hr))
		{
			//SetStatusMessage(L"No ready Kinect found!", 10000, true);
			return E_FAIL;
		}

		m_pKinectSensor = move(pKinectSensor);
		m_pCoordinateMapper = move(pCoordinateMapper);
		m_pBodyFrameReader = move(pBodyFrameReader);
		return hr;
	}

	IBodyFrameReader* BodyFrameReader() const
	{
		return m_pBodyFrameReader.Get();
	}

	ICoordinateMapper* CoordinateMapper() const
	{
		return m_pCoordinateMapper.Get();
	}

	IKinectGestureRecognizer* GestureRecongnizer() const
	{
		return nullptr;
	}

	bool HasNewFrame() const
	{
	}
	const std::list<TrackedPlayer*> &GetLatestFrame()
	{
		ComPtr<IBodyFrame> pBodyFrame;
		HRESULT hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);

		if (SUCCEEDED(hr))
		{
			INT64 nTime = 0;

			hr = pBodyFrame->get_RelativeTime(&nTime);

			IBody* ppBodies[BODY_COUNT] = { 0 };

			if (SUCCEEDED(hr))
			{
				hr = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);
			}

			if (SUCCEEDED(hr))
			{
				ProcessBody(nTime, BODY_COUNT, ppBodies);
			}

			for (int i = 0; i < _countof(ppBodies); ++i)
			{
				SafeRelease(ppBodies[i]);
			}
		}
	}

	//static void FillFrameWithIBody(Kinematics::BoneAnimationFrame& frame, IBody* pBody)
	//{
	//	Joint joints[JointType_Count];
	//	JointOrientation oris[JointType_Count];
	//	pBody->GetJoints(ARRAYSIZE(joints), joints);
	//	pBody->GetJointOrientations(ARRAYSIZE(oris), oris);
	//	oris[0].Orientation
	//};
private:
	// Current Kinect
	Microsoft::WRL::ComPtr<IKinectSensor>          m_pKinectSensor;
	Microsoft::WRL::ComPtr<ICoordinateMapper>      m_pCoordinateMapper;
	// Body reader
	Microsoft::WRL::ComPtr<IBodyFrameReader>       m_pBodyFrameReader;

	std::list<TrackedPlayer*>	m_Players;
};

bool Platform::Devices::Kinect::HasNewFrame() const
{
	return false;
}

const std::list<TrackedPlayer*>& Platform::Devices::Kinect::GetLatestFrame()
{
	// TODO: insert return statement here
}
