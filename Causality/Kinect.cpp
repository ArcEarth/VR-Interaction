#include "pch_bcl.h"
#include <Kinect.h>
#include "Kinect.h"
#include <boost\range.hpp>
#include <boost\range\adaptors.hpp>
#include <iostream>
#include <chrono>

using namespace Causality;
using namespace Causality::Devices;
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

template< typename ContainerT, typename PredicateT >
void erase_if(ContainerT& items, const PredicateT& predicate) {
	for (auto it = items.begin(); it != items.end(); ) {
		if (predicate(*it)) it = items.erase(it);
		else ++it;
	}
};

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

char* JointNames[JointType_Count] = {
	"SpineBase",
	"SpineMid",
	"Neck",
	"Head",
	"ShoulderLeft",
	"ElbowLeft",
	"WristLeft",
	"HandLeft",
	"ShoulderRight",
	"ElbowRight",
	"WristRight",
	"HandRight",
	"HipLeft",
	"KneeLeft",
	"AnkleLeft",
	"FootLeft",
	"HipRight",
	"KneeRight",
	"AnkleRight",
	"FootRight",
	"SpineShoulder",
	"HandTipLeft",
	"ThumbLeft",
	"HandTipRight",
	"ThumbRight",
};

std::unique_ptr<Causality::StaticArmature> TrackedPlayer::PlayerArmature;

class Kinect::Impl
{


public:
	size_t											LostThreshold = 60U;
	time_t											LastFrameTime = 0;
	Kinect*											pWrapper;
	DirectX::Plane									FloorPlane;
	DirectX::Matrix4x4								LocalMatrix;
	// Current Kinect
	Microsoft::WRL::ComPtr<IKinectSensor>          m_pKinectSensor;
	Microsoft::WRL::ComPtr<ICoordinateMapper>      m_pCoordinateMapper;
	// Body reader
	Microsoft::WRL::ComPtr<IBodyFrameReader>       m_pBodyFrameReader;

	std::map<uint64_t, TrackedPlayer*>	m_Players;

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
			TrackedPlayer::PlayerArmature = std::make_unique<Causality::StaticArmature>(JointType_Count, parents, JointNames);
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
		std::cout << "[DEBUG] Kinect is intialized succuessfuly!" << std::endl;
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
		::Joint joints[JointType_Count];
		JointOrientation oris[JointType_Count];

		if (SUCCEEDED(hr))
		{
			time_t nTime = 0;

			hr = pBodyFrame->get_RelativeTime(&nTime);
			//auto t = std::chrono::system_clock::from_time_t(nTime);
			//t.time_since_epoch
			if (LastFrameTime >= nTime) // Not an new frame
			{
				return false;
			}

			LastFrameTime = nTime;

			IBody* ppBodies[BODY_COUNT] = { 0 };

			if (SUCCEEDED(hr))
			{
				hr = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);
			}

			for (auto itr = m_Players.begin(), end = m_Players.end(); itr != end;)
			{
				itr->second->IsCurrentTracked = false;
				if (++(itr->second->LostFrameCount) >= (int)LostThreshold)
				{
					pWrapper->OnPlayerLost(*itr->second);
					std::cout << "Player Lost : ID = " << itr->second->PlayerID << std::endl;
					itr = m_Players.erase(itr);
				}
				else
					++itr;
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

				if (Id == 0) // This data is invaliad
					break;


				bool isNew = false;
				auto& player = m_Players[Id];
				if (player == nullptr)
				{
					player = new TrackedPlayer;
					std::cout << "New Player Tracked : ID = " << Id << std::endl;
					isNew = true;
				}

				if (!isTracked)
				{
					if (!isNew && player->IsCurrentTracked) // Maybe wait for couple of frames?
					{
						pWrapper->OnPlayerLost(*player);
						std::cout << "Player Lost : ID = " << Id << std::endl;
					}
					player->IsCurrentTracked = false;
				}
				else
				{
					player->IsCurrentTracked = true;
					player->LostFrameCount = 0;
				}

				pBody->GetJoints(ARRAYSIZE(joints), joints);
				pBody->GetJointOrientations(ARRAYSIZE(oris), oris);

				auto& frame = player->PoseFrame;

				// WARNING!!! This may be ill-formed if transform contains shear or other non-rigid transformation
				DirectX::XMVECTOR junk,rotation;
				DirectX::XMMATRIX transform = LocalMatrix;
				DirectX::XMMatrixDecompose(&junk, &rotation, &junk, transform);

				for (int j = 0; j < JointType_Count; j++)
				{
					DirectX::XMVECTOR ep = DirectX::XMLoadFloat3(reinterpret_cast<DirectX::Vector3*>(&joints[j].Position));
					frame[j].EndPostion = DirectX::XMVector3TransformCoord(ep,transform);
					ep = DirectX::XMLoadFloat4(reinterpret_cast<DirectX::Quaternion*>(&oris[j].Orientation));
					frame[j].GlobalOrientation = DirectX::XMQuaternionMultiply(ep, rotation);
				}

				auto& sk = *TrackedPlayer::PlayerArmature;
				for (const auto& jId : sk.joint_indices())
				{
					if (jId < JointType_Count)
						frame[jId].UpdateLocalData(frame[sk[jId]->ParentID()]);
					else
						// Update 'intermediate" joints if not presented in Kinect
						frame[jId].UpdateGlobalData(frame[sk[jId]->ParentID()]);
				}

				if (isNew)
				{
					pWrapper->OnPlayerTracked(*player);
					std::cout << "New Player Tracked : ID = " << Id << std::endl;
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

	//static void FillFrameWithIBody(Kinematics::BoneDisplacementFrame& frame, IBody* pBody)
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
};

std::weak_ptr<Kinect> Kinect::wpCurrentDevice;

std::shared_ptr<Kinect> Causality::Devices::Kinect::GetForCurrentView()
{
	if (wpCurrentDevice.expired())
	{
		auto pDevice = CreateDefault();
		wpCurrentDevice = pDevice;
		return pDevice;
	}
	else
	{
		return wpCurrentDevice.lock();
	}
}

Kinect::~Kinect()
{
}

bool Kinect::IsConnected() const
{
	return pImpl->IsActive();
}

void Kinect::SetDeviceCoordinate(const DirectX::Matrix4x4 & mat)
{
	pImpl->LocalMatrix = mat;
}

const DirectX::Matrix4x4 & Kinect::GetDeviceCoordinate()
{
	return pImpl->LocalMatrix;
}

void Kinect::ProcessFrame()
{
	pImpl->ProcessFrame();
}

const std::map<uint64_t, TrackedPlayer*>& Kinect::GetLatestPlayerFrame()
{
	return pImpl->m_Players;
}

// Static Constructors!!!

std::shared_ptr<Kinect> Kinect::CreateDefault()
{
	std::shared_ptr<Kinect> pKinect(new Kinect);
	if (SUCCEEDED(pKinect->pImpl->Initalize()))
		return pKinect;
	else
		return nullptr;
}

Kinect::Kinect()
:pImpl(new Impl)
{
	pImpl->pWrapper = this;
}

TrackedPlayer::TrackedPlayer()
{
	PoseFrame.resize(PlayerArmature->size());
}
