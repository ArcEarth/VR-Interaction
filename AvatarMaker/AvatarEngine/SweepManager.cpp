#include "stdafx.h"
#include "SweepManager.h"
#include "DXMathExtend.h"

using namespace Geometrics;
using namespace Kinematics;
using namespace DirectX;
using namespace std;
using namespace EditingTools;

SweepManager::SweepManager(const std::shared_ptr<Geometrics::DynamicMetaBallModel> &pTargetModel ,const std::shared_ptr<Geometrics::DynamicMetaBallModel> &pLocalModel)
	: BasicTool(pTargetModel , pLocalModel)
{}

SweepManager::~SweepManager(void)
{
}

void SweepManager::Edit(const DirectX::Vector3& pHand,const DirectX::Vector3& pWrist,const DirectX::Vector3& pElbow,const DirectX::Vector3& pShoulder)
{
	if (!IsActive()) return; 
	XMVECTOR vHand = pHand;
	XMVECTOR vShoulder = pShoulder;
	if (m_EditTrajectory.empty()) {
		m_TransformWeights = m_pTargetModel->InverseDeformMetaball(Metaball(vShoulder,Radius()));
		if (m_TransformWeights.empty())
		{
			cout<<" Not Connected , Abort"<<endl;
			Abort();
			return;
		}
		XMMATRIX Transform = m_pTargetModel->TransformMatrix(m_TransformWeights);
		XMVECTOR vDeterminant;
		Transform = XMMatrixInverse(&vDeterminant,Transform);
		if (XMVector4Equal(vDeterminant,g_XMZero))
		{
			cout<<" Not Connected , Abort"<<endl;
			Abort();
			return;
		}
		m_Core = XMVector3TransformCoord(vShoulder,Transform);
		//m_InverseTransformMatrix = Transform;

		//XMStoreFloat4x4(&m_InverseTransformMatrix,Transform);
	}

	XMMATRIX Transform = m_pTargetModel->TransformMatrix(m_TransformWeights);
	
	{
		//Transform = XMMatrixMultiply(XMLoadFloat4x4(&m_InverseTransformMatrix),Transform);
		XMMATRIX InvTransform = XMMatrixInverse(nullptr,Transform); 
		vHand = XMVector3TransformCoord(vHand,InvTransform);
	}

	m_EditTrajectory.push_back((Vector3)vHand);

	bool hr = ReshapeSurface(Transform);

	if (hr)
		m_pLocalModel->Update();
}

void SweepManager::Start()
{
	BasicTool::Start();
	m_EditTrajectory.clear();
}

void SweepManager::Finish()
{
	BasicTool::Finish();
}

bool SweepManager::ReshapeSurface(DirectX::CXMMATRIX Transform)
{
	if (m_EditTrajectory.size()<2) return false;
	m_pLocalModel->Clear();
	auto& Metaballs = m_pLocalModel->RestVolume();
	auto pTrajectory = m_EditTrajectory.FixIntervalSampling(Radius()*Metaball_Spare_Factor);
	if (!pTrajectory) return false;
	auto& Trajectory = *pTrajectory;

	XMVECTOR vCore = XMVector3TransformCoord(m_Core,Transform);

	for (unsigned int i = 0; i < pTrajectory->size(); i++)
	{
		XMVECTOR vEnd = Trajectory[i];
		vEnd = XMVector3TransformCoord(vEnd,Transform);
		Vector3 SurfacePoint;
		if (m_pTargetModel->Volume().RayIntersection(SurfacePoint,vEnd,vCore - vEnd))
		{
			XMVECTOR vBegin = (XMVECTOR)SurfacePoint;
			m_pLocalModel->AddCylinder(vBegin,vEnd,Radius(),Radius());
		}
		//BasicTool::InterpolateOnLineSegment(Flesh,St,Ed,m_Radius,m_Radius,0);
	}
	m_pLocalModel->SetDirty(DynamicMetaBallModel::VOLUME_DATA);
	return true;
}

//void SweepManager::AppendMetaballsByPoint( FXMVECTOR Ed)
//{
//	XMVECTOR St = Projection(Ed,m_pBindingBone->Entity[Default].Position,m_pBindingBone->Parent->Entity[Default].Position);
//	Vector3 SurfacePoint;
//	if (m_pTargetModel->Flesh->RayIntersection(SurfacePoint,St,Ed-St))
//		St = (XMVECTOR)SurfacePoint;
//	BasicTool::InterpolateOnLineSegment(*m_pLocalModel->Flesh,St,Ed,m_Radius,m_Radius,0);
//}
