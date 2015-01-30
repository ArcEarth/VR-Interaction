#include "stdafx.h"
#include "VolumeSculptManager.h"
#include "PrimitiveBatch.h"
#include "VertexTypes.h"
#include "Effects.h"
#include <ppl.h>
#include <DirectXColors.h>
#include "AlignedVector4.h"
#include <mutex>
#include <chrono>
#include <CommonStates.h>
#include <wrl\client.h>

using namespace DirectX;
using namespace std;
using namespace Geometrics;
using namespace Kinematics;
using namespace EditingTools;
using namespace Microsoft::WRL;
const float ApproximatePrecise = 0.08f;

const float TargetSurfaceDensity = 0.03f;

enum Faces
{
	Positive_X = 0,
	Negitive_X = 1,
	Positive_Y = 2,
	Negitive_Y = 3,
	Positive_Z = 4,
	Negitive_Z = 5,
	Front	= 5,
	Back	= 4,
	Left	= 0,
	Right	= 1,
	Top		= 2,
	Bottom	= 3,
};

XMVECTORF32 FacePatchColors[6] = {
	Colors::LawnGreen,
	Colors::RosyBrown,
	Colors::Tomato,
	Colors::Blue,
	Colors::BlueViolet,
	Colors::ForestGreen,
};
const VolumeSculptManager::CubicBezierPatch::CombinationType VolumeSculptManager::CubicBezierPatch::Combination;
const VolumeSculptManager::CubicBezierCurve::CombinationType VolumeSculptManager::CubicBezierCurve::Combination;

class VolumeSculptManager::Visual
	: public IRenderObject
{
public:
	Visual (ID3D11DeviceContext *pContext , const Geometrics::PolygonSoup<VertexPositionColor> *_pMesh, const ICamera* _pCamera , FXMVECTOR _Color);
	~Visual();
	virtual void Render(ID3D11DeviceContext *pContext);

	void clear();
public:
	XMFLOAT4 Color;
	//const Joint* pBindBone;
	//vector<VertexPositionColor> Vertices;
	//vector<std::pair<uint16_t,uint16_t>> Lines;
	const Geometrics::PolygonSoup<VertexPositionColor> *pMesh;
protected:
	unique_ptr<PrimitiveBatch<VertexPositionColor>> pDirectXBatch;
	unique_ptr<BasicEffect> pEffect;
	unique_ptr<CommonStates> pStates;
	ComPtr<ID3D11InputLayout> pInputLayout;
	const ICamera* pCamera;
};

VolumeSculptManager::Visual::Visual(ID3D11DeviceContext *pContext , const Geometrics::PolygonSoup<VertexPositionColor> *_pMesh ,const ICamera* _pCamera ,  FXMVECTOR _Color)
	: pDirectXBatch(new PrimitiveBatch<VertexPositionColor>(pContext,12288))
	, pCamera(_pCamera)
	, pMesh(_pMesh)
{
	Microsoft::WRL::ComPtr<ID3D11Device> pDevice;
	pContext->GetDevice(&pDevice);

	pEffect.reset(new BasicEffect(pDevice.Get()));
	pStates.reset(new CommonStates(pDevice.Get()));
	XMStoreFloat4(&Color,_Color);

	pEffect->SetVertexColorEnabled(true);
	//pEffect->EnableDefaultLighting();

	void const* shaderByteCode;
	size_t byteCodeLength;

	pEffect->GetVertexShaderBytecode(&shaderByteCode, &byteCodeLength);

	ThrowIfFailed(
		pDevice->CreateInputLayout(VertexPositionColor::InputElements,
		VertexPositionColor::InputElementCount,
		shaderByteCode, byteCodeLength,
		&pInputLayout)
		);
}


void VolumeSculptManager::Visual::Render(ID3D11DeviceContext *pContext)
{
	//if (Lines.size()==0)
	//	return;
	if (pMesh->empty()) return;

	pEffect->SetProjection(pCamera->GetProjectionMatrix());
	pEffect->SetView(pCamera->GetViewMatrix());
	pEffect->SetWorld(XMMatrixIdentity());
	pEffect->Apply(pContext);
	pContext->RSSetState(pStates->Wireframe());
	pContext->IASetInputLayout(pInputLayout.Get());
	//uint16_t* Indecis = reinterpret_cast<uint16_t*>(&Lines[0]);
	const uint16_t* Indecis = reinterpret_cast<const uint16_t*>(&pMesh->Faces[0]);

	pDirectXBatch->Begin();
	//pDirectXBatch->DrawIndexed(D3D11_PRIMITIVE_TOPOLOGY_LINELIST,Indecis,Lines.size()*2,&Vertices[0],Vertices.size());
	//pDirectXBatch->Draw(D3D10_PRIMITIVE_TOPOLOGY_POINTLIST,&pMesh->Vertices[0],pMesh->Vertices.size());
	pDirectXBatch->DrawIndexed(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST,Indecis,pMesh->Faces.size()*3,&pMesh->Vertices[0],pMesh->Vertices.size());
	pDirectXBatch->End();
}

VolumeSculptManager::Visual::~Visual()
{
}

void VolumeSculptManager::Visual::clear()
{
	//Vertices.clear();
	//Lines.clear();
}

VolumeSculptManager::VolumeSculptManager(ID3D11DeviceContext *pContext ,const std::shared_ptr<Geometrics::DynamicMetaBallModel> &pTargetModel ,const std::shared_ptr<Geometrics::DynamicMetaBallModel> &pLocalModel ,const ICamera* pCamera)
	: BasicTool(pTargetModel , pLocalModel)
{
	m_pVisual.reset(new Visual(pContext,&m_TargetMesh,pCamera,Colors::Aqua));
}

void VolumeSculptManager::Start()
{
	if (IsActive()) return;
	_hand_switch = false;
	BasicTool::Start();
	for (auto& curves : EditingTrajectories)
		for (auto& curve : curves)
			curve.clear();
	m_TargetMesh.clear();
	m_pVisual->clear();
}

VolumeSculptManager::~VolumeSculptManager( void )
{
}

// Once one time in two times call this function will take effect
IRenderObject* VolumeSculptManager::Visualization()
{
	static bool Flap = false;
	Flap = !Flap;
	if (Flap)
		return m_pVisual.get();
	else 
		return nullptr;
}

void VolumeSculptManager::Finish()
{
	if (!IsActive()) return ;

	IterativeApproximate();

	m_pTargetModel->Update();

	//BasicTool::Finish();
	//Clears
	m_pTargetModel->Update();
	m_pLocalModel->Clear();
	m_pVisual->clear();
	_Active = false;
}

//void VolumeSculptManager::ConstructeEndupBoundry()
//{
//	XMVECTOR Ltr = EditingTrajectories[0].back();
//	Ltr = m_pWarpper->SnapToSurface(Ltr,m_pBindingBone);
//	XMVECTOR Rtr = EditingTrajectories[0].back();
//	Rtr = m_pWarpper->SnapToSurface(Rtr,m_pBindingBone);
//	ImportTrajectoryPoints((Vector3)Ltr,(Vector3)Rtr);
//}

void VolumeSculptManager::Edit(_In_reads_(8) const DirectX::Vector3* EditingPoints)
{
	if (EditingTrajectories[0][0].empty()) {
		XMVECTOR CPoint = (static_cast<XMVECTOR>(EditingPoints[7]) + static_cast<XMVECTOR>(EditingPoints[3])) * 0.5f;
		m_TransformWeights = m_pTargetModel->InverseDeformMetaball(Metaball(CPoint,Radius()));
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
	}

	XMMATRIX Transform = m_pTargetModel->TransformMatrix(m_TransformWeights);

	{
		XMMATRIX InvTransform = XMMatrixInverse(nullptr,Transform);
		for (size_t arm = 0; arm < 2; arm++)
		{
			for (size_t joint = 0; joint < 4; joint++)
			{
				XMVECTOR vJoint = EditingPoints[arm*4 + joint];
				XMVector3TransformCoord(vJoint,InvTransform);
				EditingTrajectories[arm][joint].push_back(static_cast<Vector3>(vJoint));
			}
		}
	}

	bool hr = ReshapeVolume(Transform);

	if (hr)
		m_pLocalModel->Update();

}

void VolumeSculptManager::Edit(const DirectX::Vector3& p1,const DirectX::Vector3& p2 ,const DirectX::Vector3& p3 ,const DirectX::Vector3& p4)
{
	if (!IsActive()) return;
	static DirectX::Vector3 _buffer[8];
	if (_hand_switch)
	{
		_buffer[4] = p1;
		_buffer[5] = p2;
		_buffer[6] = p3;
		_buffer[7] = p4;
		Edit(_buffer);
	}else
	{
		_buffer[0] = p1;
		_buffer[1] = p2;
		_buffer[2] = p3;
		_buffer[3] = p4;
	}
	_hand_switch = !_hand_switch;
}

const float Threshold = 0.05f; 

void VolumeSculptManager::IterativeApproximate()
{
	auto st = std::chrono::system_clock::now();
	Geometrics::MetaBallModel& Volume = m_pLocalModel->RestVolume();
	const Geometrics::MetaBallModel& Subject = m_pTargetModel->Volume();

	unsigned int MaxIterateTimes = 10;
	
	size_t i = 0;
	for (; i < MaxIterateTimes; i++)
	{
		ApproximateVolum(Volume, Subject);
		if (Volume.Primitives.empty()) 
			break;
		m_pTargetModel->AddVolume(Volume);
		m_pTargetModel->Update();
	}
	auto ed = std::chrono::system_clock::now();
	auto dur = ed-st;
	cout << dur.count() <<" milliseconds , " << i << " iterations" <<endl;
}
//
//void VolumeSculptManager::AppendToTargetSurface(FXMVECTOR Lp, const FXMVECTOR Rp )
//{
//	//const Vector3 ProjectionDirection(0.0f,0.0f,1.0f);
//	XMVECTOR vProj = g_XMIdentityR2;
//	vProj = XMVector3InverseRotate(vProj,m_pBindingBone->Entity[Current].Orientation);
//	vProj = XMVector3Rotate(vProj,m_pBindingBone->Entity[Default].Orientation);
//	Vector3 ProjectionDirection(vProj);
//	Vector3 Cp;
//	XMVECTOR _Sp ,_Lp = Lp,_Rp = Rp;
//	_Sp = FindCircumSphereCentre_TwoPoint_ApicalAngle_ProjectDirection((Vector3)Lp, (Vector3)Rp ,ProjectionDirection,XM_2PI*0.333333333f);
//	float delta = 3.0f/(float)(TargetSurfaceTesselation-1);
//	float t = -1.0f;
//	for (int i = 0; i < TargetSurfaceTesselation; i++ , t += delta)
//	{
//		Cp = Slerp(_Sp,_Lp,_Rp,t);
//		m_TargetSurface[i].push_back(Cp);
//	}
//	return;
//}

void VolumeSculptManager::ApproximateVolum(_Out_ Geometrics::MetaBallModel &Volume,_In_ const Geometrics::MetaBallModel &Subject )
{
	//Clears
	Volume.clear();

	std::vector<Vector4A,AlignedAllocator<Vector4A>> SurfaceGrid;
	const float EffectiveRatio = Subject.EffictiveRadiusRatio();
	const size_t offset = TargetSurfaceTesselation * (TargetSurfaceTesselation / 2);
	//Travel the surface to build a grid of surface point
	XMVECTOR vSp = Subject[0].Position;

	std::mutex vectormutex;

#ifdef _DEBUG
	std::for_each
#else
	concurrency::parallel_for_each
#endif
		(m_TargetMesh.Vertices.cbegin(),m_TargetMesh.Vertices.cend(),[&](const DirectX::VertexPositionColor & vertex)
	{

		XMVECTOR vCp = XMLoadFloat3(&vertex.position);

		XMVECTOR RayDirction = vSp-vCp;

		//We don't need to do the sphere insertion since it's has been inside
		if (Subject.Contains(vCp)) {
			return; // seems like it's never so easy to avoid this exception
		}

		//The mesh is represented by the implicit function,so you need to get the original one from caller
		Vector3 SurfaceP(vSp);
		int c = 0;
		bool hr  = Subject.RayIntersection(SurfaceP,vCp,RayDirction);

		if (!hr) {
			//	cout<<"["<<setw(2)<<j<<","<<setw(2)<<i<<"] : "<<"exception : Can't find projection point."<<endl;
			return;
		}

		hr = Inside(m_TargetMesh,SurfaceP);
		if (!hr) 
		{
			//cout<<"Projected Point " << SurfaceP << " outside bounding volume."<<endl;
			return;
		}
		//Directly using the surface point is not good
		//SurfaceP *=0.99f;
		//				SurfaceP += RayDirction*0.03;

		float Cd = Geometrics::Distance(m_TargetMesh,SurfaceP);

		//Cd /= EffectiveRatio;
		if (Cd > ApproximatePrecise)
		{
			{
				std::lock_guard<mutex> guard(vectormutex);
				SurfaceGrid.emplace_back(SurfaceP.x,SurfaceP.y,SurfaceP.z,Cd);
			}
			//cout<<"Sphere : {" << SurfaceP << ","<< Cd <<"} added"<<endl;
		}
	});

	if (SurfaceGrid.empty()) return;

	std::sort(SurfaceGrid.begin(),SurfaceGrid.end(),[](const Vector4A &lhs,const Vector4A&rhs){
		return lhs.w < rhs.w;
	});

	//Find the max Inscribe Sphere and add to the joint , until max Inscribe Sphere 's radius < 0.03
	while (true)
	{
		auto maxball = std::max_element(SurfaceGrid.begin(),SurfaceGrid.end(),[](const Vector4A &lhs,const Vector4A&rhs){
			return lhs.w < rhs.w;
		});

		if (maxball->w<ApproximatePrecise) break;

		Metaball ThisBall(*maxball);
		ThisBall.Radius /= EffectiveRatio;
		Volume.push_back(ThisBall);
		//cout<<"Selected Sphere : {" << ThisBall.Position <<" , "<< ThisBall.Radius << "}"<<endl;

		for (auto& ball : SurfaceGrid)
		{
			XMVECTOR vTp = ThisBall.Position;
			vTp -= ball;
			vTp = XMVector3Length(vTp);
			float dis = XMVectorGetX(vTp);
			if (dis - ball.w < ball.w)
				ball.w = dis - ball.w;
			//ball.w = std::min<float>(ball.w , dis - maxball->w);
		}
		//for (unsigned int i = 0; i < SurfaceGrid.size(); i++)
		//{
		//	XMVECTOR vTp = ThisBall.Position;
		//	vTp -= SurfaceGrid[i];
		//	vTp = XMVector3Length(vTp);
		//	float dis = XMVectorGetX(vTp);
		//	if (dis + SurfaceGrid[i].w < maxball->w)
		//		SurfaceGrid[i].w = 0;
		//	//SurfaceGrid[i].w = min(SurfaceGrid[i].w , dis - maxball->w);
		//}

		maxball->w = 0.0f; // remove maxball from the process
	}
}

XMVECTOR PointReflect(FXMVECTOR V,FXMVECTOR Origin)
{
	return Origin*g_XMTwo - V;
}

void VolumeSculptManager::TesselateBezierPatch(const VolumeSculptManager::CubicBezierPatch &patch, size_t tessellation, FXMVECTOR Color)
{
	size_t offset = m_TargetMesh.Vertices.size();

	for (size_t i = 0; i <= tessellation; i++)
	{
		float u = (float)i / tessellation;

		auto clipping = patch.row_clipping(u);
		for (size_t j = 0; j <= tessellation; j++)
		{
			float v = (float)j / tessellation;
			auto position = clipping(v);
			m_TargetMesh.Vertices.emplace_back(position,Color);
		}
	}

	size_t stride = tessellation + 1;
	for (size_t i = 0; i < tessellation; i++)
	{
		for (size_t j = 0; j < tessellation; j++)
		{
			// Make a list of six index values (two triangles).
			m_TargetMesh.Faces.emplace_back(
				offset + i * stride + j,
				offset + (i + 1) * stride + j,
				offset + (i + 1) * stride + j + 1);
			m_TargetMesh.Faces.emplace_back(
				offset + i * stride + j,
				offset + (i + 1) * stride + j + 1,
				offset + i * stride + j + 1);
		}
	}

}


bool VolumeSculptManager::ReshapeVolume(DirectX::CXMMATRIX Transform)
{
	if (EditingTrajectories[0][0].size() < 4) return false;
	auto pLH = EditingTrajectories[0][0].FixCountSampling2(4);
	auto pRH = EditingTrajectories[1][0].FixCountSampling2(4);
	auto pLW = EditingTrajectories[0][1].FixCountSampling2(4);
	auto pRW = EditingTrajectories[1][1].FixCountSampling2(4);
	auto pLE = EditingTrajectories[0][2].FixCountSampling2(4);
	auto pRE = EditingTrajectories[1][2].FixCountSampling2(4);
	auto pLS = EditingTrajectories[0][3].FixCountSampling2(4);
	auto pRS = EditingTrajectories[1][3].FixCountSampling2(4);
	auto& LH = *pLH;
	auto& LW = *pLW;
	auto& LE = *pLE;
	auto& LS = *pLS;
	auto& RH = *pRH;
	auto& RW = *pRW;
	auto& RE = *pRE;
	auto& RS = *pRS;
	for (size_t i = 0; i < 4; i++)
	{
		LH[i] = XMVector3TransformCoord(LH[i],Transform);
		LE[i] = XMVector3TransformCoord(LE[i],Transform);
		LW[i] = XMVector3TransformCoord(LW[i],Transform);
		LS[i] = XMVector3TransformCoord(LS[i],Transform);
		RH[i] = XMVector3TransformCoord(RH[i],Transform);
		RE[i] = XMVector3TransformCoord(RE[i],Transform);
		RW[i] = XMVector3TransformCoord(RW[i],Transform);
		RS[i] = XMVector3TransformCoord(RS[i],Transform);
	}
	std::vector<Vector3> LWr(4);
	std::vector<Vector3> RWr(4);
	std::vector<Vector3> LSr(4);
	std::vector<Vector3> RSr(4);
	for (size_t i = 0; i < 4; i++)
	{
		LWr[i] = PointReflect(LW[i],LH[i]);
		RWr[i] = PointReflect(RW[i],RH[i]);
		LSr[i] = PointReflect(LS[i],LE[i]);
		RSr[i] = PointReflect(RS[i],RE[i]);
	}
	//     |T|
	// |B|L|F|R|
	//     |B|
	for (size_t i = 0; i < 4; i++)
	{
		Patches[Left][i][0] = LE[i];
		Patches[Left][i][1] = LSr[i];
		Patches[Left][i][2] = LW[i];
		Patches[Left][i][3] = LH[i];

		Patches[Front][i][0] = LH[i];
		Patches[Front][i][1] = LWr[i];
		Patches[Front][i][2] = RWr[i];
		Patches[Front][i][3] = RH[i];

		Patches[Right][i][0] = RH[i];
		Patches[Right][i][1] = RW[i];
		Patches[Right][i][2] = RSr[i];
		Patches[Right][i][3] = RE[i];

		Patches[Back][i][0] = RE[i];
		Patches[Back][i][1] = RS[i];
		Patches[Back][i][2] = LS[i];
		Patches[Back][i][3] = LE[i];
	}

	// To-do : better interploate for top/bottom patch
	{
		for (size_t i = 0; i < 4; i++)
		{
			Patches[Top][0][i] = Patches[Back][0][3-i];
			Patches[Top][1][i] = PointReflect(Patches[Back][1][3-i], Patches[Back][0][3-i]);
			Patches[Top][2][i] = PointReflect(Patches[Front][1][i], Patches[Front][0][i]);
			Patches[Top][3][i] = Patches[Front][0][i];
			Patches[Bottom][0][i] = Patches[Front][3][i];
			Patches[Bottom][1][i] = PointReflect(Patches[Front][2][i], Patches[Front][3][i]);
			Patches[Bottom][2][i] = PointReflect(Patches[Back][2][3-i], Patches[Back][3][3-i]);
			Patches[Bottom][3][i] = Patches[Back][3][3-i];
		}

		for (size_t i = 1; i < 3; i++)
		{
			Patches[Top][i][0] = Patches[Left][0][i];
			//Patches[Top][i][1] += PointReflect(Patches[Left][1][i], Patches[Left][0][i]);
			//Patches[Top][i][2] += PointReflect(Patches[Right][1][3-i], Patches[Right][0][3-i]);
			Patches[Top][i][3] = Patches[Right][0][3-i];
			Patches[Bottom][i][0] = Patches[Left][3][3-i];
			//Patches[Bottom][i][1] += PointReflect(Patches[Left][2][3-i], Patches[Left][3][3-i]);
			//Patches[Bottom][i][2] += PointReflect(Patches[Right][2][i], Patches[Right][3][i]);
			Patches[Bottom][i][3] = Patches[Right][3][i];
		}

		//for (size_t i = 0; i < 4; i++)
		//	for (size_t j = 0; j < 4; j++)
		//	{
		//		Patches[Top][i][j] *= 0.5f;
		//		Patches[Bottom][i][j] *= 0.5f;
		//	}
	}

	m_TargetMesh.clear();
	for (size_t i = 0; i < 6; i++)
	{
		TesselateBezierPatch(Patches[i],15,FacePatchColors[i]);
	}

	return true;

	//	m_TargetCenterAxis.clear();
	//
	//	const auto Row = TargetSurfaceTesselation;
	//	const auto Col = TargetSurfaceTesselation;
	//
	//	const size_t verticalSegments = TargetSurfaceTesselation / 4;
	//	const size_t horizontalSegments = TargetSurfaceTesselation;
	//
	//	XMVECTOR vProj = g_XMIdentityR2;
	//	//vProj = XMVector3InverseRotate(vProj,m_pBindingBone->Entity[Current].Orientation);
	//	//vProj = XMVector3Rotate(vProj,m_pBindingBone->Entity[Default].Orientation);
	//	float RadiusAngle = XM_2PI*0.333333333f;
	//
	//	XMFLOAT3A EquatorVertices[Col];
	//
	//	float delta = XM_2PI/(float)(Col);
	//	float theta = 0;
	//
	//	for (int i = 0; i < Col; i++)
	//	{
	//		XMScalarSinCos(&EquatorVertices[i].x,&EquatorVertices[i].z,theta);
	//		EquatorVertices[i].y = 0;
	//		theta += delta;
	//	}
	//
	//	for (int i = 0; i < Row; i++)
	//	{
	//		XMVECTOR Lp = pLT->at(i) , Rp = pRT->at(i);
	//		Vector3 ProjectionDirection(vProj);
	//		XMVECTOR Sp = FindCircumSphereCentre_TwoPoint_ApicalAngle_ProjectDirection((Vector3)Lp, (Vector3)Rp ,ProjectionDirection,RadiusAngle);
	//		m_TargetCenterAxis.emplace_back(Sp);
	//		XMVECTOR XAxis = XMVector3Normalize(Lp - Sp);
	//		XMVECTOR ZAxis = XMVector3Normalize(Rp - Sp);
	//		XMVECTOR YAxis = XMVector3Cross(XAxis , ZAxis);
	//		ZAxis = XMVector3Cross(XAxis,YAxis);
	//
	//		XMMATRIX mTransform(XAxis,YAxis,ZAxis,g_XMIdentityR3);
	//		//mTransform = XMMatrixTranspose(mTransform);
	//		mTransform = XMMatrixScalingFromVector(XMVector3Length(Lp-Sp)) * mTransform;
	//		mTransform *= XMMatrixTranslationFromVector(Sp);
	//
	//		if (i == 0 || i == Row-1) 
	//		{
	//			float deltaLatitude = -XM_PIDIV2/verticalSegments;
	//			float latitude = i==0?XM_PIDIV2:0;
	//			for (size_t j = 0; j <= verticalSegments; ++j)
	//			{
	//				float dy, dxz;
	//
	//				XMScalarSinCos(&dy, &dxz, latitude);
	//
	//				// Create a single ring of vertices at this latitude.
	//				for (size_t k = 0; k < horizontalSegments; k++)
	//				{
	//					float u = (float)k / horizontalSegments;
	//
	//					float longitude = k * XM_2PI / horizontalSegments;
	//					float dx, dz;
	//
	//					XMScalarSinCos(&dx, &dz, longitude);
	//
	//					dx *= dxz;
	//					dz *= dxz;
	//
	//					XMVECTOR vSp = XMVectorSet(dx, dy, dz, 0);
	//					vSp = XMVector3Transform(vSp,mTransform);
	//
	//					m_TargetMesh.Vertices.emplace_back(vSp,Colors::AliceBlue);
	//				}
	//				latitude += deltaLatitude;
	//			}
	//		} else
	//		{
	//			for (int j = 0; j < Col; j++)
	//			{
	//				XMVECTOR Cp = XMLoadFloat3A(&EquatorVertices[j]);
	//				Cp = XMVector3Transform(Cp,mTransform);
	//				m_TargetMesh.Vertices.emplace_back(Cp,Colors::AliceBlue);
	//			}
	//		}
	//
	//
	//
	//		//m_TargetMesh.Faces.emplace_back(GetIndex(i,j),GetIndex(i,(j+1)%Col),GetIndex(i+1,(j+1)%Col));
	//		//m_TargetMesh.Faces.emplace_back(GetIndex(i,j),GetIndex(i+1,(j+1)%Col),GetIndex(i+1,j));
	//
	//		//float delta = 2.0f/(float)(Col-1);
	//		//float t = -1.0f;
	//		//for (int j = 0; j < Col; j++ , t += delta)
	//		//{
	//		//	Cp = Slerp(Sp,Lp,Rp,t);
	//		//	m_TargetMesh.Vertecis.push_back(Cp);
	//		//	m_TargetMesh.Faces.emplace_back(GetIndex(i,j),GetIndex(i,(j+1)%Col),GetIndex(i+1,(j+1)%Col));
	//		//	m_TargetMesh.Faces.emplace_back(GetIndex(i,j),GetIndex(i+1,(j+1)%Col),GetIndex(i+1,j));
	//		//}
	//	}
	//
	//#define GetIndex(row,colume) (((row)*Col)+(colume))
	//	for (size_t i = 0; i < Row + verticalSegments*2 - 1; i++)
	//	{
	//		// Create a single ring of vertices at this latitude.
	//		for (size_t j = 0; j < horizontalSegments; j++)
	//		{
	//			m_TargetMesh.Faces.emplace_back(GetIndex(i,j),GetIndex(i,(j+1)%Col),GetIndex(i+1,(j+1)%Col));
	//			m_TargetMesh.Faces.emplace_back(GetIndex(i,j),GetIndex(i+1,(j+1)%Col),GetIndex(i+1,j));
	//		}
	//	}
	//#undef GetIndex
	//
	//
	//	//int N =  m_TargetSurface[0].size();
	//	if (m_TargetMesh.empty()) return;
	//	m_pVisual->clear();
}

//DirectX::Vector3 VolumeSculptManager::FindTendencyPointOnSurface( const DirectX::Vector3 &p0,const DirectX::Vector3 &p1,const DirectX::Vector3 &p2,const DirectX::Vector3 &defaultTarget )
//{
//	Vector3 v0 = p0 - p2;
//	Vector3 v1 = p0 - p1;
//	Vector3 v2 = p1 - p2;
//	Vector3 Ray0 = v1 , Ray1 , Ray2 , Ray3 , Ps ;
//
//	v1.Normalize();
//	v2.Normalize();
//	v0.Normalize();
//
//	Ray1 = v1^v2;
//	Ray1 = Ray1^v0;
//
//	Ray0.Normalize();
//	Ray1.Normalize();
//
//	Ray2 = -Ray1;
//
//	Ray3 = defaultTarget - p0;
//	Ray3.Normalize();
//
//	//Find a point on surface
//	auto pFlesh = this->m_pLocalModel->Flesh;
//	if (!pFlesh->FindRayIntersectionPointWithMesh(Ps,p0,Ray0)) {
//		if (!pFlesh->FindRayIntersectionPointWithMesh(Ps,p0,Ray1)) {
//			if (!pFlesh->FindRayIntersectionPointWithMesh(Ps,p0,Ray2)) {
//				if (!pFlesh->FindRayIntersectionPointWithMesh(Ps,p0,Ray3)) {
//					Ps.x = Ps.y = Ps.z =0.0f;
//				}
//			}
//		}
//	}
//
//	return Ps;
//}
