#include "stdafx.h"
#include "Player.h"
#include <DirectXColors.h>
#include "DXMathExtend.h"

using namespace DirectX;
using namespace Kinematics;

Player::Player(ID3D11DeviceContext* pContext ,const ICamera* pCamera , FXMVECTOR IndicatorColor)
	: m_pSkeleton(new HumanSkeleton)
	, m_pVisualizer(nullptr)
	, m_Tracked(true)
	, m_Visiable(true)
	, m_Enable(true)
	, m_TrackStartTime(0)
{
	InitializeVisualizer(pContext,pCamera,IndicatorColor);
}

Player::Player()
	: m_pSkeleton(new HumanSkeleton)
	, m_pVisualizer(nullptr)
	, m_Tracked(true)
	, m_Visiable(true)
	, m_Enable(true)
	, m_TrackStartTime(0)
{
}

void Player::InitializeVisualizer(ID3D11DeviceContext* pContext ,const ICamera* pCamera , FXMVECTOR IndicatorColor)
{
	m_pVisualizer.reset(new SkeletonViewer(pContext,pCamera,m_pSkeleton.get(),HumanSkeleton::HumanBoneColorSchedule));
	m_pVisualizer->SelectFlag.Specify(Kinematics::Current);
	m_pVisualizer->Color = IndicatorColor;
	m_pVisualizer->Color.w = 0.2f;
	//m_pVisualizer->SelectFlag+=Kinematics::Default;
}

bool Player::GestureHolder::Update(const MultiMouseControler& Mice)
{
	if (DeviceIndex == -1) return false;
	auto state = Mice[DeviceIndex].State;
	if ((Gesture != Decoder[state]) && ((Mice[DeviceIndex].SteadyTimer.GetTime() > 300) || Mice.GetTimeFromLastUpdate() > 150))
	{
		Gesture = Decoder[state];
		Trigger = true;
	}
	return Trigger;
}

bool Player::PlayerHand::UpdateGestures(const MultiMouseControler& Mice)
{
	bool UpdateAvailable = false;
	for (auto& holder : GestureHolders)
	{
		holder.Update(Mice);
	}

	for (auto& holder : GestureHolders)
	{
		if (holder.Trigger)
		{
			return true;
		} else if (holder.Gesture != GestureEnum::FREE)
		{
			return false;
		}
	}
	return false;
}


unsigned int Player::UpdateGestures(const MultiMouseControler& Mice)
{
	return (unsigned int)Hands[0].UpdateGestures(Mice) | (unsigned int)Hands[1].UpdateGestures(Mice) << 1;
}

float Player::Velocity() const
{
	float distance = 0.0f;
	if (TimeStamps.size() < 2 || (TimeStamps.front() - m_TrackStartTime) < 1000) return 0.0f;
	std::vector<Vector3> Trajectory(BodyCenterTrajectory.cbegin(),BodyCenterTrajectory.cend());
	SpaceCurveSampler::LaplaianSmoothing(Trajectory);
	for (size_t i = 1; i < TimeStamps.size(); i++)
	{
		XMVECTOR vDisplacement = (Trajectory[i] - Trajectory[i-1]);
		distance += Vector3::Length(vDisplacement);
	}
	float timeInterval = static_cast<float>(TimeStamps.back() - TimeStamps.front()) / 1000.0f ;
	return distance / timeInterval;
}

float Player::HandVelovity(unsigned int wichHand) const
{
	float distance = 0.0f;
	if (TimeStamps.size() < 2) return 0.0f;
	for (size_t i = 1; i < TimeStamps.size(); i++)
	{
		XMVECTOR vDisplacement = (Hands[wichHand].Trajectory[i] - Hands[wichHand].Trajectory[i-1]);

		// Relative speed to body center
		vDisplacement -= (BodyCenterTrajectory[i] - BodyCenterTrajectory[i-1]);
		distance += Vector3::Length(vDisplacement);
	}
	float timeInterval = static_cast<float>(TimeStamps.back() - TimeStamps.front()) / 1000.0f ;
	return distance / timeInterval;
}



Player::~Player(void)
{
}