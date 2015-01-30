#include "stdafx.h"
#include "DragManager.h"
#include <Eigen\Sparse>
//#include "DXMathExtend.h"
//#include <iostream>

using namespace DirectX;
using namespace Kinematics;
using namespace std;
using namespace EditingTools;
using namespace Geometrics;
using namespace boost;

static const float DEFAULT_PEAK_RADIUS = 0.03f;

DragManager::DragManager(const std::shared_ptr<Geometrics::DynamicMetaBallModel> &pTargetModel ,const std::shared_ptr<Geometrics::DynamicMetaBallModel> &pLocalModel)
	: BasicTool(pTargetModel , pLocalModel)
{
	m_PeakRadius = DEFAULT_PEAK_RADIUS;
}


DragManager::~DragManager(void)
{
}

void DragManager::Start(){
	BasicTool::Start();
	m_EditTrajectory.clear();
	m_AppendingFlag = true;
}


void DragManager::Finish()
{
	if (!IsActive()) return;
	if (m_pLocalModel->RestVolume().size()>0) 
	{
		auto retessaltion =	[this](MetaBallModel &Volume,Eigen::MatrixXf& Weights){
			SpaceCurveSampler Sampler;
			for (const auto& ball : Volume.Primitives)
				Sampler.push_back(ball.Position);
			float Interval = Radius()*Metaball_Spare_Factor;
			auto pResampledCurve = Sampler.FixIntervalSampling(Interval);

			// Resample volume
			MetaBallModel Volume_R;
			Volume_R.ISO = Volume.ISO;
			const float iso = Volume.ISO;

			for (auto& point : *pResampledCurve)
				Volume_R.Primitives.emplace_back(point,Radius());
			const auto N = Volume.size();
			const auto M = Volume_R.size();
			const auto BoneCount = Weights.cols();

			// Resample volue weights
			//ConnectionGraph g(Volume.size()+Volume_R.size());
			vector<float> w_v(Volume_R.size(),0.0f);

			Eigen::MatrixXf B(M,BoneCount);
			B.setZero();

			for (size_t u = 0; u < N; u++)
				for (size_t v = 0; v < M; v++)
					if (Metaball::Connected(Volume[u],Volume_R[v],iso))
					{
						float w = Metaball::ConnectionStrength(Volume[u],Volume_R[v],iso);
						w_v[v] += w;
						B.row(v) += Weights.row(u) * w;
					}

			auto Laplace = CreateLaplacianMatrix(Volume_R,w_v);
			Eigen::SimplicialCholesky<Eigen::SparseMatrix<float>> Cholesky(Laplace);

			//cout<<Weights<<endl;;
			Weights = Cholesky.solve(B);
			//cout<<Weights<<endl;
			Volume = Volume_R;
		};
		float R = Radius();
		SetRadius(R / 15.0f);
		XMMATRIX Transform = m_pTargetModel->TransformMatrix(m_TransformWeights);
		ReshapePath(Transform);
		m_pTargetModel->AddVolume(m_pLocalModel->RestVolume(),retessaltion);
		SetRadius(R);
		m_pTargetModel->Update();
		m_pLocalModel->Clear();
		m_pLocalModel->Update();
	}

	else Abort();
}

void DragManager::Edit(const DirectX::Vector3& pHand,const DirectX::Vector3& pWrist,const DirectX::Vector3& pElbow,const DirectX::Vector3& pShoulder){
	if (!IsActive()) return; 
	Edit(pHand);
}

void DragManager::Edit(const Vector3& editPoint)
{
	XMVECTOR Rp = editPoint;
	if (m_EditTrajectory.empty()) {
		m_TransformWeights = m_pTargetModel->InverseDeformMetaball(Metaball(editPoint,Radius()));
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
		//m_InverseTransformMatrix = Transform;

		//XMStoreFloat4x4(&m_InverseTransformMatrix,Transform);
	}

	XMMATRIX Transform = m_pTargetModel->TransformMatrix(m_TransformWeights);
	
	{
		//Transform = XMMatrixMultiply(XMLoadFloat4x4(&m_InverseTransformMatrix),Transform);
		XMMATRIX InvTransform = XMMatrixInverse(nullptr,Transform); 
		Rp = XMVector3TransformCoord(Rp,InvTransform);
	}

	m_EditTrajectory.push_back((Vector3)Rp);

	bool hr = ReshapePath(Transform);

	if (hr)
		m_pLocalModel->Update();

}

bool DragManager::ReshapePath(DirectX::CXMMATRIX Transform){
	// Custom Sampling
	float r = m_EditTrajectory.length();
	float Interval = Radius()*Metaball_Spare_Factor;
	auto pTrajectory = m_EditTrajectory.FixIntervalSampling(Interval);
	//auto pTrajectory = m_EditTrajectory.FixCountSampling(Count);

	if (!pTrajectory) return false;

	auto& Trajectory = *pTrajectory;

	auto& Metaballs = m_pLocalModel->RestVolume();
	Metaballs.clear();

	//XMVECTOR LastP = g_XMInfinity;
	for (unsigned int i =0 ; i<Trajectory.size();i++)
	{
		XMVECTOR pt = Trajectory[i];
		pt = XMVector3TransformCoord(pt,Transform);
		Metaballs.Primitives.emplace_back((Vector3)pt,Radius(),1);
		//LastP = pt;
	}

	m_pLocalModel->SetDirty(DynamicMetaBallModel::VOLUME_DATA);
	return true;

#ifdef DEBUG
	//std::cout<<"Ball [0]	: "<<Metaballs[0].Position<<std::endl;
	//std::cout<<"Ball [end]	: "<<Metaballs.back().Position<<std::endl<<"*******************"<<std::endl;
#endif // DEBUG
}
