#include "pch_bcl.h"
#include <Kinect.h>
#include "Kinect.h"
#include <boost\range.hpp>
#include <boost\range\adaptors.hpp>
#include <iostream>
#include <chrono>
#include <wrl\event.h>
#include "ArmatureBlock.h"

using namespace Causality;
using namespace Causality::Devices;
using namespace Microsoft::WRL;

extern BlockArmature g_PlayeerBlocks;
extern bool			 g_EnableInputFeatureLocalization;


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

JointType KinectSensor::JointsParent[JointType_Count] = {
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

std::unique_ptr<Causality::StaticArmature> TrackedBody::BodyArmature;

struct FilteredPlayer : public TrackedBody
{
	std::array<Vector3DynamicFilter, JointType_Count> JointFilters;
};

class KinectSensor::Impl
{


public:
	size_t											LostThreshold = 60U;
	time_t											LastFrameTime = 0;
	float											FrameRate = 30;
	bool											m_EnableMirrow = false;

	KinectSensor*									pWrapper;
	DirectX::Plane									FloorPlane;
	DirectX::Matrix4x4								LocalMatrix;

	// Current Kinect
	Microsoft::WRL::ComPtr<IKinectSensor>			m_pKinectSensor;
	Microsoft::WRL::ComPtr<ICoordinateMapper>		m_pCoordinateMapper;
	// Body reader
	Microsoft::WRL::ComPtr<IBodyFrameSource>		m_pBodyFrameSource;
	Microsoft::WRL::ComPtr<IBodyFrameReader>		m_pBodyFrameReader;
	Microsoft::WRL::Wrappers::Event					m_FrameReadyEvent;

	std::list<TrackedBody>							m_Players;
	std::unique_ptr<std::thread>					m_pThread;

	Impl()
	{

	}
public:
	bool IsActive() const
	{
		return m_pKinectSensor;
	}

	HRESULT Initalize()
	{

		if (TrackedBody::BodyArmature == nullptr)
		{
			int parents[JointType_Count];
			std::copy_n(KinectSensor::JointsParent, (int) JointType_Count, parents);
			TrackedBody::BodyArmature = std::make_unique<Causality::StaticArmature>(JointType_Count, parents, JointNames);
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

		m_pKinectSensor = move(pKinectSensor);

		return hr;

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
		}

		if (!pKinectSensor || FAILED(hr))
		{
			std::cout << "[DEBUG] Failed to intialize KINECT" << std::endl;
			//SetStatusMessage(L"No ready Kinect found!", 10000, true);
			return hr;
		}

		m_pKinectSensor = move(pKinectSensor);
		m_pBodyFrameSource = move(pBodyFrameSource);
		m_pCoordinateMapper = move(pCoordinateMapper);
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

	bool StartTracking()
	{
		WAITABLE_HANDLE hBFEvent = NULL;
		HRESULT hr;
		BOOLEAN active = FALSE;

		if (m_pKinectSensor)
		{
			// Initialize the Kinect and get coordinate mapper and the body reader

			hr = m_pKinectSensor->Open();

			if (SUCCEEDED(hr))
			{
				hr = m_pKinectSensor->get_BodyFrameSource(&m_pBodyFrameSource);
			}
		}

		if (!m_pKinectSensor || FAILED(hr))
		{
			std::cout << "[DEBUG] Failed to intialize KINECT" << std::endl;
			//SetStatusMessage(L"No ready Kinect found!", 10000, true);
			return hr;
		}

		std::cout << "[DEBUG] Kinect is intialized succuessfuly!" << std::endl;

		hr = m_pBodyFrameSource->OpenReader(&m_pBodyFrameReader);

		if (FAILED(hr))
			return false;

		m_pBodyFrameSource->get_IsActive(&active);
		if (!active)
			return false;

		hr = m_pBodyFrameReader->SubscribeFrameArrived(&hBFEvent);

		if (FAILED(hr))
			return false;

		m_FrameReadyEvent.Attach(reinterpret_cast<HANDLE>(hBFEvent));

		m_pBodyFrameReader->put_IsPaused(FALSE);

		m_pThread = std::make_unique<std::thread>([this](){
			while (m_FrameReadyEvent.IsValid())
			{
				auto result = WaitForSingleObject(m_FrameReadyEvent.Get(), 0);
				switch (result)
				{
				case WAIT_ABANDONED:
					throw;
				case WAIT_FAILED:
					m_FrameReadyEvent.Detach();
					return;
				case WAIT_TIMEOUT:
					continue;
				case WAIT_OBJECT_0:
					break;
				};

				ComPtr<IBodyFrameArrivedEventArgs> args;
				ComPtr<IBodyFrameReference> frameRef;
				ComPtr<IBodyFrame> pBodyFrame;

				HRESULT hr = m_pBodyFrameReader->GetFrameArrivedEventData(reinterpret_cast<WAITABLE_HANDLE>(m_FrameReadyEvent.Get()), &args);
				if (FAILED(hr))
					continue;
				hr = args->get_FrameReference(&frameRef);
				if (FAILED(hr))
					continue;
				hr = frameRef->AcquireFrame(&pBodyFrame);
				if (FAILED(hr))
					continue;

				ProcessFrame(pBodyFrame.Get());
			}
		});
		return true;
	}

	void StopTracking()
	{
		if (!m_FrameReadyEvent.IsValid())
			return;

		m_pBodyFrameReader->UnsubscribeFrameArrived(reinterpret_cast<WAITABLE_HANDLE>(m_FrameReadyEvent.Get()));
		m_FrameReadyEvent.Detach();
		if (m_pThread)
			m_pThread->join();
	}

	bool HasNewFrame() const
	{
	}

	bool ProcessFrame()
	{
		ComPtr<IBodyFrame> pBodyFrame;
		HRESULT hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);

		if (FAILED(hr))
		{
			return false;
		}

		return ProcessFrame(pBodyFrame.Get());
	}

	bool ProcessFrame(IBodyFrame* pBodyFrame)
	{
		time_t nTime = 0;

		HRESULT hr = pBodyFrame->get_RelativeTime(&nTime);

		if (FAILED(hr))
		{
			return false;
		}

		//auto t = std::chrono::system_clock::from_time_t(nTime);
		//t.time_since_epoch
		if (LastFrameTime >= nTime) // Not an new frame
		{
			return false;
		}

		LastFrameTime = nTime;

		// Process experied bodies
		for (auto itr = m_Players.begin(), end = m_Players.end(); itr != end;)
		{
			itr->m_IsTracked = false;
			if (++(itr->m_LostFrameCount) >= (int)LostThreshold)
			{
				pWrapper->OnPlayerLost(*itr);
				std::cout << "Player Lost : ID = " << itr->Id << std::endl;
				if (itr->RefCount <= 0)
					itr = m_Players.erase(itr);
				else
					++itr;
			}
			else
				++itr;
		}

		// Retrive new bodies
		IBody* ppBodies[BODY_COUNT] = { 0 };

		hr = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);

		if (FAILED(hr))
		{
			return false;
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

			TrackedBody* player = nullptr;
			bool isNew = false;

			//FilteredPlayer* player = static_cast<FilteredPlayer*>(m_Players[Id]);
			auto& itr = std::find_if(m_Players.begin(), m_Players.end(), [Id](const TrackedBody& body)->bool { return body.Id == Id;});

			if (itr != m_Players.end())
				player = &(*itr);

			if (isTracked)
			{
				// New Player tracked
				if (player == nullptr)
				{
					m_Players.emplace_back();
					player = &m_Players.back();
					player->m_pKinectSensor = this->pWrapper;
					player->Id = Id;
					player->m_IsTracked = isTracked;

					std::cout << "New Player Detected : ID = " << Id << std::endl;
					isNew = true;
				}

				//std::cout << "Player Tracked, ID = '" << Id << '\'' << std::endl;

				UpdateTrackedBodyData(player, pBody, LastFrameTime, isNew);
			}
			else if (player != nullptr) // Handel Losted player
			{
				if (player->m_LostFrameCount > 3)
				{
					//for (auto& filter : player->JointFilters)
					//{
					//	filter.Reset();
					//}
				}

				if (!isNew && player->m_IsTracked) // Maybe wait for couple of frames?
				{
					pWrapper->OnPlayerLost(*player);
					std::cout << "Player Lost : ID = " << Id << std::endl;
				}
				player->m_IsTracked = false;
			}

			//SafeRelease(ppBodies[i]);
		}

		return true;
	}

	void UpdateTrackedBodyData(Causality::TrackedBody * player, IBody * pBody, time_t time, bool isNew = false)
	{
		::Joint joints[JointType_Count];
		JointOrientation oris[JointType_Count];

		const static DirectX::XMVECTORF32 g_XMNegateXZ = { -1.0f, 1.0f, -1.0f, 1.0f };

		player->m_IsTracked = true;
		player->m_LostFrameCount = 0;

		pBody->GetJoints(ARRAYSIZE(joints), joints);
		pBody->GetJointOrientations(ARRAYSIZE(oris), oris);
		player->m_Distance = joints[0].Position.Z;

		TrackedBody::FrameType frame(JointType::JointType_Count);
		//frame.Time = std::chrono::system_clock::from_time_t(time).time_since_epoch();

		// WARNING!!! This may be ill-formed if transform contains shear or other non-rigid transformation
		DirectX::XMVECTOR junk, rotation;
		DirectX::XMMATRIX transform = LocalMatrix;

		// Flip X and Z, X is due to mirrow effect, Z is due to Kinect uses LH coordinates
		if (m_EnableMirrow)
			transform.r[0] = DirectX::XMVectorNegate(transform.r[0]);
		//transform.r[2] = DirectX::XMVectorNegate(transform.r[2]);

		// Extract the rotation to apply
		DirectX::XMMatrixDecompose(&junk, &rotation, &junk, transform);

		for (int j = 0; j < JointType_Count; j++)
		{
			//auto& filter = player->JointFilters[j];
			DirectX::XMVECTOR ep = DirectX::XMLoadFloat3(reinterpret_cast<DirectX::Vector3*>(&joints[j].Position));
			ep = DirectX::XMVector3TransformCoord(ep, transform);
			DirectX::XMVECTOR bp = DirectX::XMLoadFloat3(reinterpret_cast<DirectX::Vector3*>(&joints[JointsParent[j]].Position));
			//frame[j].GblLength = DirectX::XMVectorGetX(DirectX::XMVector3Length(ep - bp));
			//frame[j].LclLength = frame[j];

			//? Why not filtering?
			//! We are tracking gestures here, keep the raw data is better!
			frame[j].GblTranslation = ep; // filter.Apply(ep);
			ep = DirectX::XMLoadFloat4(reinterpret_cast<DirectX::Quaternion*>(&oris[j].Orientation));

			ep *= DirectX::g_XMNegateX;
			//ep *= g_XMNegateXZ.v;

			frame[j].GblRotation = DirectX::XMQuaternionMultiply(ep, rotation);
		}

		auto& armature = *TrackedBody::BodyArmature;

		for (auto jId : armature.joint_indices())
		{
			// Update 'intermediate" joints if not presented in Kinect
			auto pId = armature[jId]->ParentID();
			if (jId > JointType_Count)
			{
				frame[jId].UpdateGlobalData(frame[pId]);
			}
			DirectX::XMVECTOR q = frame[jId].GblRotation.LoadA();
			if (DirectX::XMVector4Equal(q, DirectX::XMVectorZero()))
			{
				if (pId != -1)
					q = frame[pId].GblRotation.LoadA();
				else
					q = DirectX::XMQuaternionIdentity();
				frame[jId].GblRotation.StoreA(q);
			}

		}

		frame.RebuildLocal(armature);

		player->PushFrame(std::move(frame));

		if (isNew)
		{
			pWrapper->OnPlayerTracked(*player);
			std::cout << "New Player Tracked : ID = " << player->Id << std::endl;
		}
		else
		{

			//player->OnFrameArrived(*player, frame);
			HandState state;
			pBody->get_HandLeftState(&state);
			if (state != player->m_HandStates[0])
			{
				player->m_HandStates[0] = state;
				//player->OnHandStateChanged(*player, HandType_LEFT, state);
			}
			pBody->get_HandRightState(&state);
			if (state != player->m_HandStates[1])
			{
				player->m_HandStates[1] = state;
				//player->OnHandStateChanged(*player, HandType_RIGHT, state);
			}
		}
	}

	//static void FillFrameWithIBody(Kinematics::AffineFrame& frame, IBody* pBody)
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

std::weak_ptr<KinectSensor> KinectSensor::wpCurrentDevice;

std::shared_ptr<KinectSensor> Causality::Devices::KinectSensor::GetForCurrentView()
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

KinectSensor::~KinectSensor()
{
}

bool KinectSensor::IsConnected() const
{
	return pImpl->IsActive();
}

void KinectSensor::SetDeviceCoordinate(const DirectX::Matrix4x4 & mat)
{
	pImpl->LocalMatrix = mat;
}

void KinectSensor::EnableMirrowing(bool enable_mirrow)
{
	pImpl->m_EnableMirrow = enable_mirrow;
}

bool Causality::Devices::KinectSensor::IsMirrowingEnable() const
{
	return pImpl->m_EnableMirrow;
}

const DirectX::Matrix4x4 & KinectSensor::GetDeviceCoordinate()
{
	return pImpl->LocalMatrix;
}

void KinectSensor::ProcessFrame()
{
	pImpl->ProcessFrame();
}

bool Causality::Devices::KinectSensor::Start()
{
	BufferedStreamViewer<int> buffer;
	decltype(buffer) buffer2(buffer);

	return pImpl->StartTracking();
}

void Causality::Devices::KinectSensor::Stop()
{
	pImpl->StopTracking();
}

const std::list<TrackedBody>& KinectSensor::GetTrackedBodies() const
{
	return pImpl->m_Players;
}
std::list<TrackedBody>& KinectSensor::GetTrackedBodies()
{
	return pImpl->m_Players;
}

// Static Constructors!!!

std::shared_ptr<KinectSensor> KinectSensor::CreateDefault()
{
	std::shared_ptr<KinectSensor> pKinect = std::make_shared<KinectSensor>();
	if (SUCCEEDED(pKinect->pImpl->Initalize()))
		return pKinect;
	else
		return nullptr;
}

KinectSensor::KinectSensor()
	:pImpl(new KinectSensor::Impl)
{
	pImpl->pWrapper = this;
}

//TrackedBody::TrackedBody()
//{
//	//PoseFrame.resize(BodyArmature->size());
//}

//Causality::TrackedBody::TrackedBody(const TrackedBody &)
//{
//}

Causality::TrackedBody::TrackedBody(size_t bufferSize)
	: RefCount(0) , Id(0), m_IsTracked(false), m_LastTrackedTime(0), m_LostFrameCount(0), m_FrameBuffer(bufferSize)
{
}

void Causality::TrackedBody::PushFrame(FrameType && frame)
{
	m_FrameBuffer.Push(std::move(frame));
	m_FrameBuffer.LockBuffer();
	auto pFrame = m_FrameBuffer.PeekLatest();
	if (!pFrame)
	{
		m_FrameBuffer.UnlockBuffer();
		return;
	}

	assert(pFrame != nullptr); // since we just pushed one frame 
	OnFrameArrived(*this, *pFrame);
	m_FrameBuffer.UnlockBuffer();
}

Causality::AffineFrame & Causality::TrackedBody::PullLatestFrame() const
{
	return *m_FrameBuffer.MoveToLatest();
}

Causality::AffineFrame & Causality::TrackedBody::CurrentFrame() const
{
	return *m_FrameBuffer.GetCurrent();
}

float Causality::TrackedBody::DistanceToSensor() const
{
	return m_Distance;
}

void Causality::TrackedBody::AddRef() {
	++RefCount;
}

void Causality::TrackedBody::Release() {
	--RefCount;
}

Causality::TrackedBodySelector::TrackedBodySelector(KinectSensor * pKinect, SelectionMode mode)
	:pCurrent(nullptr), pKinect(nullptr), mode(None)
{
	Initialize(pKinect, mode);
}

Causality::TrackedBodySelector::~TrackedBodySelector()
{
	if (pKinect)
	{
		con_tracked.disconnect();
		con_lost.disconnect();
	}
}

void Causality::TrackedBodySelector::OnPlayerTracked(TrackedBody & body)
{
	if ((!pCurrent || !(mode & Sticky)))
	{
		if (!pCurrent || ( mode == SelectionMode::Closest && body.DistanceToSensor() < pCurrent->DistanceToSensor()))
		{
			ChangePlayer(&body);
		}
	}
}

void Causality::TrackedBodySelector::OnPlayerLost(TrackedBody & body)
{
	if (pCurrent && body == *pCurrent)
	{
		ReSelectFromAllTrackedBodies();
	}
}

void Causality::TrackedBodySelector::ReSelectFromAllTrackedBodies()
{
	TrackedBody *pBestPlayer = nullptr;

	if (mode & Closest)
	{
		float distance = 100000;
		for (auto& player : pKinect->GetTrackedBodies())
		{
			if (player.IsTracked() && player != *pCurrent && distance > player.DistanceToSensor())
			{
				pBestPlayer = &player;
				distance = player.DistanceToSensor();
			}
		}
	}
	else // Eariest tracked player
	{
		for (auto& player : pKinect->GetTrackedBodies())
		{
			if (player.IsTracked() && player != *pCurrent)
			{
				pBestPlayer = &player;
				break;
			}
		}
	}

	ChangePlayer(pBestPlayer);

}

void Causality::TrackedBodySelector::Reset() {
	fpTrackedBodyChanged(pCurrent, nullptr);
	if (pCurrent)
		pCurrent->Release();
	pCurrent = nullptr;
}

void Causality::TrackedBodySelector::Initialize(Devices::KinectSensor * pKinect, SelectionMode mode)
{
	this->mode = mode;
	if (pKinect)
	{
		this->pKinect = pKinect->GetRef();
		con_tracked =
			pKinect->OnPlayerTracked += MakeEventHandler(&TrackedBodySelector::OnPlayerTracked, this);
		con_tracked = 
			pKinect->OnPlayerLost += MakeEventHandler(&TrackedBodySelector::OnPlayerLost, this);
	}
}

void Causality::TrackedBodySelector::ChangePlayer(TrackedBody * pNewPlayer)
{
	auto pOld = pCurrent;
	con_frame.disconnect();
	pCurrent = pNewPlayer;

	if (pCurrent)
	{
		pCurrent->AddRef();
		con_frame = pCurrent->OnFrameArrived.connect(fpFrameArrived);
	}

	fpTrackedBodyChanged(pOld, pCurrent);

	if (pOld)
		pOld->Release();
}

void Causality::TrackedBodySelector::SetFrameCallback(const FrameEventFunctionType & callback) {
	fpFrameArrived = callback;
	if (con_frame.connected())
	{
		con_frame.disconnect();
		con_frame = pCurrent->OnFrameArrived.connect(fpFrameArrived);
	}
}

void Causality::TrackedBodySelector::SetPlayerChangeCallback(const PlayerEventFunctionType & callback) { 
	fpTrackedBodyChanged = callback; 
}

void Causality::TrackedBodySelector::ChangeSelectionMode(SelectionMode mdoe)
{
	this->mode = mode;
	if (pCurrent)
		ReSelectFromAllTrackedBodies();
}
