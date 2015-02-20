#include <Kinect.h>
#include "Kinect.h"
#include <boost\range.hpp>
#include <boost\range\adaptors.hpp>
using namespace Platform::Devices;
using namespace Microsoft::WRL;

template <class T>
inline void SafeRelease(T*& pCom)
{
	if (pCom != nullptr)
	{
		pCom->Release();
		pCom = nullptr;
	}
}

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

std::unique_ptr<Armatures::ArmatureBase> TrackedPlayer::PlayerArmature;

class Kinect::Impl
{
public:
	bool IsActive() const
	{
		return m_pKinectSensor;
	}

	HRESULT Initalize()
	{
		if (TrackedPlayer::PlayerArmature == nullptr)
		{
			int parents[JointType_Count];
			std::copy_n(Kinect::JointsParent,(int)JointType_Count, parents);
			TrackedPlayer::PlayerArmature = std::make_unique<Armatures::StaticArmature>(JointType_Count, parents);
		}
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

		if (pKinectSensor)
		{
			// Initialize the Kinect and get coordinate mapper and the body reader

			hr = pKinectSensor->Open();

			if (SUCCEEDED(hr))
			{
				hr = pKinectSensor->get_CoordinateMapper(&pCoordinateMapper);
			}

			if (SUCCEEDED(hr))
			{
				hr = pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);
			}

			if (SUCCEEDED(hr))
			{
				hr = pBodyFrameSource->OpenReader(&pBodyFrameReader);
			}
		}

		if (!pKinectSensor || FAILED(hr))
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

	bool ProcessFrame()
	{
		ComPtr<IBodyFrame> pBodyFrame;
		HRESULT hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);
		Joint joints[JointType_Count];
		JointOrientation oris[JointType_Count];

		if (SUCCEEDED(hr))
		{
			INT64 nTime = 0;

			hr = pBodyFrame->get_RelativeTime(&nTime);

			IBody* ppBodies[BODY_COUNT] = { 0 };

			if (SUCCEEDED(hr))
			{
				hr = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);
			}

			for (int i = 0; i < _countof(ppBodies); ++i)
			{
				auto &pBody = ppBodies[i];
				if (pBody == nullptr)
					break;

				UINT64 Id;
				pBody->get_TrackingId(&Id);
				BOOLEAN isTracked;
				pBody->get_IsTracked(&isTracked);


				bool isNew = false;
				auto& player = m_Players[Id];
				if (player == nullptr)
				{
					player = new TrackedPlayer;
					isNew = true;
				}

				if (!isTracked)
				{
					if (!isNew && player->IsCurrentTracked) // Maybe wait for couple of frames?
					{
						pWrapper->OnPlayerLost(*player);
					}
					player->IsCurrentTracked = isTracked;
				}

				pBody->GetJoints(ARRAYSIZE(joints), joints);
				pBody->GetJointOrientations(ARRAYSIZE(oris), oris);

				auto& frame = player->CurrentFrame;
				for (int j = 0; j < JointType_Count; j++)
				{
					frame[j].EndPostion = reinterpret_cast<DirectX::Vector3&>(joints[j].Position);
					frame[j].GlobalOrientation = reinterpret_cast<DirectX::Quaternion&>(oris[j].Orientation);
				}

				auto& sk = *TrackedPlayer::PlayerArmature;
				for (const auto& jId : sk.TopologyOrder)
				{
					if (jId < JointType_Count)
						frame[jId].UpdateLocalData(frame[sk.ParentsMap[jId]]);
					else
						// Update 'intermediate" joints if not presented in Kinect
						frame[jId].UpdateGlobalData(frame[sk.ParentsMap[jId]]);

				}

				if (isNew)
				{
					pWrapper->OnPlayerTracked(*player);
				}
				else
				{
					player->OnPoseChanged(*player);
					HandState state;
					pBody->get_HandLeftState(&state);
					if (state != player->HandStates[0])
					{
						player->HandStates[0] = state;
						player->OnHandStateChanged(*player, HandType_LEFT, state);
					}
					pBody->get_HandRightState(&state);
					if (state != player->HandStates[1])
					{
						player->HandStates[1] = state;
						player->OnHandStateChanged(*player, HandType_RIGHT, state);
					}
				}

				SafeRelease(ppBodies[i]);
			}
		}
		else
		{
			return false;
		}
		return true;
	}

	//static void FillFrameWithIBody(Kinematics::BoneAnimationFrame& frame, IBody* pBody)
	//{
	//	Joint joints[JointType_Count];
	//	JointOrientation oris[JointType_Count];
	//	pBody->GetJoints(ARRAYSIZE(joints), joints);
	//	pBody->GetJointOrientations(ARRAYSIZE(oris), oris);
	//	oris[0].Orientation
	//};

	//auto TrackedPlayers() const
	//{
	//	using namespace boost;
	//	using namespace boost::adaptors;
	//	return m_Players | map_values;
	//}

public:
	Kinect*											pWrapper;
	// Current Kinect
	Microsoft::WRL::ComPtr<IKinectSensor>          m_pKinectSensor;
	Microsoft::WRL::ComPtr<ICoordinateMapper>      m_pCoordinateMapper;
	// Body reader
	Microsoft::WRL::ComPtr<IBodyFrameReader>       m_pBodyFrameReader;

	std::map<uint64_t, TrackedPlayer*>	m_Players;
};

Platform::Devices::Kinect::~Kinect()
{
}

HRESULT Platform::Devices::Kinect::Initalize()
{
	return pImpl->Initalize();
}

bool Platform::Devices::Kinect::IsConnected() const
{
	return pImpl->IsActive();
}

void Platform::Devices::Kinect::ProcessFrame()
{
	pImpl->ProcessFrame();
}

const std::map<uint64_t, TrackedPlayer*>& Platform::Devices::Kinect::GetLatestPlayerFrame()
{
	return pImpl->m_Players;
}

// Static Constructors!!!

std::unique_ptr<Kinect> Platform::Devices::Kinect::CreateDefault()
{
	std::unique_ptr<Kinect> pKinect(new Kinect);
	if (SUCCEEDED(pKinect->Initalize()))
		return pKinect;
	else
		return nullptr;
}

Platform::Devices::Kinect::Kinect()
:pImpl(new Impl)
{
	pImpl->pWrapper = this;
}

Platform::Devices::TrackedPlayer::TrackedPlayer()
{
	CurrentFrame.resize(PlayerArmature->size());
}
