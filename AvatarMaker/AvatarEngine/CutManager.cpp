#include "stdafx.h"
#include "CutManager.h"
#include "PrimitiveBatch.h"
#include "VertexTypes.h"
#include "Effects.h"
#include <DirectXColors.h>

using namespace EditingTools;
using namespace Geometrics;
using namespace DirectX;
using namespace Kinematics;
using namespace std;

class GridMesh
{
public:
	GridMesh(unsigned int _RowCount = 0 , unsigned int _CloumnCount = 0);
	~GridMesh();

	float distance(DirectX::FXMVECTOR) const;

	std::vector<DirectX::Vector3>& operator[] (unsigned int row);

	unique_ptr<std::vector<std::pair<uint16_t,uint16_t>>> LineConnections() const;

	void add_column(const std::vector<DirectX::Vector3>& new_column);
	void add_row(const std::vector<DirectX::Vector3>& new_row);
private:
	std::vector<std::vector<DirectX::Vector3>> Entity;
};

GridMesh::GridMesh( unsigned int _RowCount /*= 0 */, unsigned int _CloumnCount /*= 0*/ )
{

}

GridMesh::~GridMesh()
{
}

class CutManager::Visual
	: public IRenderObject
{
public:
	Visual (ID3D11DeviceContext *pContext ,const ICamera* _pCamera , vector<Vector3> *_TargetSurface ,FXMVECTOR _Color);
	~Visual();
	virtual void Render(ID3D11DeviceContext *pContext);

	void clear();
public:
	XMFLOAT4 Color;
	const Joint* pBindBone;
	vector<VertexPositionColor> Vertices;
	vector<std::pair<uint16_t,uint16_t>> Lines;
protected:
	unique_ptr<PrimitiveBatch<VertexPositionColor>> pDirectXBatch;
	unique_ptr<BasicEffect> pEffect;
	ID3D11InputLayout* pInputLayout;
	vector<Vector3> *TargetSurface;
	const ICamera* pCamera;
};

CutManager::Visual::Visual(ID3D11DeviceContext *pContext ,const ICamera* _pCamera , vector<Vector3> *_TargetSurface  , FXMVECTOR _Color)
	: pDirectXBatch(new PrimitiveBatch<VertexPositionColor>(pContext))
	, TargetSurface(_TargetSurface)
	, pCamera(_pCamera)
{
	XMStoreFloat4(&Color,_Color);
		Microsoft::WRL::ComPtr<ID3D11Device> pDevice;
		pContext->GetDevice(&pDevice);
		pEffect.reset(new BasicEffect(pDevice.Get()));
	pEffect->SetVertexColorEnabled(true);
	//	pContext->OMSetBlendState();
	//	pEffect->SetAlpha(0.5f);

	void const* shaderByteCode;
	size_t byteCodeLength;

	pEffect->GetVertexShaderBytecode(&shaderByteCode, &byteCodeLength);

	pDevice->CreateInputLayout(VertexPositionColor::InputElements,
		VertexPositionColor::InputElementCount,
		shaderByteCode, byteCodeLength,
		&pInputLayout);
}


void CutManager::Visual::Render(ID3D11DeviceContext *pContext)
{
	if (Lines.size()==0)
		return;

	pEffect->SetProjection(pCamera->GetProjectionMatrix());
	pEffect->SetView(pCamera->GetViewMatrix());
	if (pBindBone)
		pEffect->SetWorld(pBindBone->BlendMatrix());
	else
		pEffect->SetWorld(XMMatrixIdentity());
	pEffect->SetWorld(XMMatrixIdentity());
	pEffect->Apply(pContext);

	pContext->IASetInputLayout(pInputLayout);
	uint16_t* Indecis = reinterpret_cast<uint16_t*>(&Lines[0]);

	pDirectXBatch->Begin();
	pDirectXBatch->DrawIndexed(D3D11_PRIMITIVE_TOPOLOGY_LINELIST,Indecis,Lines.size()*2,&Vertices[0],Vertices.size());
	pDirectXBatch->End();
}

CutManager::Visual::~Visual()
{
	SafeRelease(pInputLayout);
}

void CutManager::Visual::clear()
{
	Vertices.clear();
	Lines.clear();
	pBindBone = nullptr;
}

CutManager::CutManager( ID3D11DeviceContext *pContext,const std::shared_ptr<Geometrics::DynamicMetaBallModel> &pTargetModel ,const ICamera* pCamera )
	: m_pVisual(new Visual(pContext,pCamera,nullptr,Colors::Pink))
	, m_pTargetModel(pTargetModel)
	//, m_pWarpper(pSpaceWarpper)
{
}

CutManager::~CutManager(void)
{
}

void CutManager::Start()
{
	if (IsActive()) return;
	m_cutedFlag = false;
	_Active = true;
	m_Trajectories[0].clear();
	m_Trajectories[1].clear();
	m_pVisual->clear();
	m_pVisual->pBindBone = nullptr;
	m_DirtyFlag = 0;
}

void CutManager::Finish()
{
	if (!IsActive()) return;
	_Active = false;
	if (m_DirtyFlag > 0)
	{
		m_pTargetModel->Update();
	}
	//if (CutSubject())
	//	m_pTargetModel->Tessellate();
}

void CutManager::Abort()
{
	if (!IsActive()) return;
	_Active = false;
}

void CutManager::Edit( const DirectX::Vector3& pHand,const DirectX::Vector3& pWrist,const DirectX::Vector3& pElbow,const DirectX::Vector3& )
{
	if (!IsActive()) return;
	m_Trajectories[0].push_back(pHand);
	m_Trajectories[1].push_back(pElbow);
	ContructCutsurfaceFromTrajectory();
	if (CutSubject())
	{
		//if (m_DirtyFlag < 1) 
		//	m_DirtyFlag++;
		//else
		//{
		m_cutedFlag = true;
		m_pTargetModel->Update();
		//	m_DirtyFlag = 0;
		//}
	}
}

float CutManager::Radius() const
{
	return 0.0f;
}

bool CutManager::SetRadius( float Radius )
{
	return false;
}

bool CutManager::TurnRadius( float deltaRadius )
{
	return false;
}

DirectX::IRenderObject* CutManager::Visualization()
{
	return m_pVisual.get();
}

#define AvaliableTrajectory 10
#define AlphaDelta 0.1f
void CutManager::ContructCutsurfaceFromTrajectory()
{
	if (m_Trajectories[0].size() <= 4) return;
	auto pST = m_Trajectories[0].FixIntervalSampling(0.02f);
	auto pET = m_Trajectories[1].FixCountSampling(pST->size()-1);

	unsigned int N = min(pST->size(),pET->size());
	unsigned int S = max(0U,N - AvaliableTrajectory);
	float delta = 0.8f/(CuttingCurvTessellation-1);
	m_pVisual->clear();

	for (int i = 0; i < CuttingCurvTessellation; i++)
		m_CuttingSurface[i].clear();

	for (unsigned int k = S; k < N; k++)
	{
		XMVECTOR v0 = pST->at(k);
		XMVECTOR v1 = pET->at(k);
		float t = 0.0f;
		for (unsigned int i = 0; i < CuttingCurvTessellation ; i++,t+=delta)
		{
			XMVECTOR v = XMVectorLerp(v0,v1,t);
			m_CuttingSurface[i].push_back((Vector3)v);
			XMFLOAT4A color((float*)&m_pVisual->Color);
			color.w *= 1.0f - (k-S)*AlphaDelta;
			m_pVisual->Vertices.push_back(VertexPositionColor((Vector3)v,color));
			int Id = m_pVisual->Vertices.size() - 1;
			if (m_pVisual->Vertices.size()>CuttingCurvTessellation) 
				m_pVisual->Lines.push_back(std::pair<int,int>(Id-CuttingCurvTessellation,Id));
			if (i>0)
				m_pVisual->Lines.push_back(std::pair<int,int>(Id - 1,Id));
		}
	}
}
const float RazorThinckness = 0.8f;

bool ComputeRelayBones(Joint* pJoint,vector<bool>& bonesRelay)
{
	bool ChildrenRelay = false;
	for (auto& subJoint : pJoint->Children)
	{
		ChildrenRelay |= ComputeRelayBones(subJoint,bonesRelay);
	}

	ChildrenRelay = bonesRelay[pJoint->ID] || ChildrenRelay;
	bonesRelay[pJoint->ID] = ChildrenRelay;
	return ChildrenRelay;
}


bool CutManager::CutSubject()
{
	auto& Flesh = m_pTargetModel->RestVolume();
	auto& AFlesh = m_pTargetModel->Volume();
	std::vector<Metaball> buffer;
	std::vector<unsigned int> indexBuffer;
	const float EffictiveRatio = m_pTargetModel->RestVolume().EffictiveRadiusRatio();

	// Cut down metaballs 
	bool Result = false;

	// (--map.end())->first should be the max Key in map

	Result = m_pTargetModel->RemoveMetaball_If([&](const unsigned int index)->bool{
		float Dis = 10000.0f;
		for (int j = 0; j < CuttingCurvTessellation; j++)
		{
			Dis = min(Dis,FindDistancePointToPath(AFlesh[index].Position,m_CuttingSurface[j]));
		}
		bool Cuted = Dis < Flesh[index].Radius * EffictiveRatio * RazorThinckness;
		return Cuted;
	});

	return Result;

	//for (unsigned int i = 0; i < Flesh.size(); i++)
	//{
	//	float Dis = 10000.0f;
	//	for (int j = 0; j < CuttingCurvTessellation; j++)
	//	{
	//		Dis = min(Dis,FindDistancePointToPath(AFlesh[i].Position,m_CuttingSurface[j]));
	//	}
	//	bool Cuted = Dis < Flesh[i].Radius * EffictiveRatio * RazorThinckness;
	//	if (!Cuted) {
	//		buffer.emplace_back(Flesh[i]);
	//		indexBuffer.push_back(i);
	//	} else
	//	{
	//		Result = true;
	//	}
	//}

	//if (!Result) return false;

	//Flesh.Primitives = buffer;

	//Flesh.OptimizeConnection(0);
	//m_pTargetModel->SetDirty(DynamicMetaBallModel::VOLUME_DATA);
	//return true;

	//Flesh.Primitives = buffer;

	//// Cut down bones
	//std::vector<bool> bonesRely((--Skeleton.Index.end())->first + 1);
	//for (unsigned int i = 0; i < bonesRely.size(); i++)
	//	bonesRely[i] = false;
	//for (auto& ball : Flesh.Primitives)
	//{
	//	bonesRely[ball.BindingIndex] = true;
	//}

	//ComputeRelayBones(Skeleton.Root,bonesRely);

	//Result = false;

	//for (unsigned int i = 0; i < bonesRely.size(); i++)
	//{
	//	auto ptr = Skeleton[i];
	//	if (ptr!=nullptr && !bonesRely[i] && !ptr->IsRoot) {
	//		Result = true;
	//		Skeleton.RemoveJoint(ptr);
	//	}
	//}

	//if (!Result) return true;

	//buffer.clear();
	//for (Metaball& ball : Flesh.Primitives)
	//{
	//	if (Skeleton.containtsKey(ball.BindingIndex))
	//		buffer.push_back(ball);
	//}

	// Let it go....
	//auto mapper = Skeleton.OptimizeIndex();
	//for (auto& ball : Flesh.Primitives)
	//{
	//	ball.BindingIndex = mapper[ball.BindingIndex];
	//}

}
