#include "stdafx.h"
#include "DriveManager.h"

using namespace Kinematics;
using namespace EditingTools;

DriveManager::DriveManager(const std::shared_ptr<Geometrics::DynamicMetaBallModel> &pTargetModel , const std::shared_ptr<Kinematics::HS_MB_Adaptor> pHumanAdaptor , unsigned int HandSwitch)
	: m_pTargetModel(pTargetModel) , m_pAdptor(pHumanAdaptor) , m_HandSwitch(HandSwitch)
{
}

DriveManager::~DriveManager(void)
{
}

bool DriveManager::IsAttached() const
{
	if (!m_pTargetModel->Adaptor() || !m_pAdptor) return false;

	bool HandAttachedJoint	= m_pAdptor->IsAttached(HumanSkeleton::HAND_LEFT	+ m_HandSwitch*4);
	bool WristAttachedJoint = m_pAdptor->IsAttached(HumanSkeleton::WRIST_LEFT + m_HandSwitch*4);
	bool ElbowAttachedJoint = m_pAdptor->IsAttached(HumanSkeleton::ELBOW_LEFT + m_HandSwitch*4);

	return HandAttachedJoint || WristAttachedJoint || ElbowAttachedJoint;
}

bool DriveManager::Attach()
{
	if (!m_pTargetModel->Adaptor() || !m_pAdptor) return false;

	auto IShoulder = (HumanSkeleton::SHOULDER_LEFT + m_HandSwitch * 4);
	for (size_t i = 0; i < 4; i++)
	{
		auto joint = IShoulder+i;
		m_pAdptor->Detach(joint);
		m_pAdptor->Attach(joint);
	}
	m_pTargetModel->Update();
	return IsAttached();
}

void DriveManager::Detach()
{
	auto& weights = m_pTargetModel->VolumeWeights();

	auto IShoulder = (HumanSkeleton::SHOULDER_LEFT + m_HandSwitch * 4);
	for (size_t i = 0; i < 4; i++)
	{
		auto joint = IShoulder+i;
		m_pAdptor->Detach(joint);
	}
	m_pTargetModel->Update();
}


void DriveManager::Start()
{
	Attach();
}

void DriveManager::Finish()
{
	Detach();
}

void DriveManager::Abort()
{
}

void DriveManager::Edit( const DirectX::Vector3&,const DirectX::Vector3&,const DirectX::Vector3&,const DirectX::Vector3& )
{
}

bool DriveManager::IsActive() const
{
	return IsAttached();
}

DirectX::IRenderObject* DriveManager::Visualization()
{
	return nullptr;
}
