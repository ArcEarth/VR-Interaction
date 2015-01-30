#include "stdafx.h"
#include "FreeCursor.h"
#include <GeometricPrimitive.h>
#include <iostream>
#include "DXMathExtend.h"

using namespace EditingTools;
using namespace DirectX;

static const DirectX::XMVECTORF32 SnapedIndicator = {0.196078449f, 0.803921640f, 0.196078449f, 1.000000000f};
static const DirectX::XMVECTORF32 UnSnapedIndicator = {1.000000000f, 0.498039246f, 0.313725501f, 1.000000000f};

class FreeCursor::Preview
	: public IRenderObject
{
public:
	Preview(ID3D11DeviceContext *pContext ,const ICamera *pCamera , SphereLightData* pLight)
		: m_pCamera(pCamera)
		, m_pLight(pLight)
	{
		m_pSphere = GeometricPrimitive::CreateSphere(pContext,2.0f);
		//m_pCylinder = GeometricPrimitive::CreateCylinder(pContext);
		//pBindBone = nullptr;
	}

	void Render(ID3D11DeviceContext *pContext){
		// The rendering Radius....
		float radius = Radius();
		XMMATRIX World = XMMatrixScaling(radius,radius,radius) * XMMatrixTranslationFromVector((XMVECTOR)Position());
		XMMATRIX View = m_pCamera->GetViewMatrix();
		XMMATRIX Projection = m_pCamera->GetProjectionMatrix();

		auto tColor = Color();
		tColor.w = 0.5f;
		m_pSphere->Draw(World,View,Projection,(XMVECTOR)tColor);

	/*	if (pBindBone != nullptr) 
		{
			XMVECTOR vtrP = Position();
			XMVECTOR vtrJ = pBindBone->Entity[Kinematics::Current].Position;
			if (pBindBone->Parent!=nullptr){
				XMVECTOR vtrB = pBindBone->Parent->Entity[Kinematics::Current].Position;
				vtrJ = FindProjectionPointToSegment(vtrP,vtrJ,vtrB);
			}

			vtrP -= vtrJ;

			if (XMVector3NearEqual(vtrP, g_XMZero , g_XMEpsilon * 10.0f))
				return;
			float Length = Vector3::Length(vtrP);
			XMVECTOR Orientation = Quaternion::RotationQuaternion(g_XMIdentityR1,vtrP);
			XMMATRIX World = XMMatrixTranslation(0.0f,0.5f,0.0f) * XMMatrixScaling(0.02f,Length - radius * 0.75f ,0.02f)
				* XMMatrixRotationQuaternion(Orientation) * XMMatrixTranslationFromVector(vtrJ);
			
			XMMATRIX View = m_pCamera->GetViewMatrix();
			XMMATRIX Projection = m_pCamera->GetProjectionMatrix();

			m_pCylinder->Draw(World,View,Projection,(XMVECTOR)Color());
		}*/
	}

	Vector4& Color() {return m_pLight->Color; }
	Vector3& Position() {return m_pLight->Position; }
	float&	Radius() {return m_pLight->Radius;}

public:
	//const Kinematics::Joint* pBindBone;

protected:
	const ICamera		*m_pCamera;
	SphereLightData		*m_pLight;

	std::unique_ptr<GeometricPrimitive> m_pSphere;
	//std::unique_ptr<GeometricPrimitive> m_pCylinder;
};

IRenderObject* FreeCursor::Visualization()
{
	if (!Snaped)
		return m_pPreview.get();
	return nullptr;
}

bool FreeCursor::SetRadius(float Radius){
	m_pPreview->Radius() = Radius;
	return true;
}

//void Edit(const std::vector<HolographicPoint>& EditingPoints)
//{
//	auto Hp = EditingPoints[0].Indices[0];
//	Kinematics::Joint* pBindBone = nullptr;
//	auto Point = m_pWarpper->Warp_Bind_Closest_Bone_And_Inverse_Animation_Transform(pHand,&pBindBone);
//	Point = m_pWarpper->Warp_Snap_To_Surface(Point,pBindBone);
//	Point = m_pWarpper->Warp_Animation_Transform(Point);
//
//
//	if (!pBindBone)
//	{
//		std::cout<<"Exception : Can't find binding bone";
//	}
//
//	Update(Point,pBindBone);
//}


void FreeCursor::Edit(const DirectX::Vector3& pHand,const DirectX::Vector3&,const DirectX::Vector3&,const DirectX::Vector3&)
{
	//const Kinematics::Joint* pBindBone = nullptr;
	//auto Point = m_pWarpper->Warp_Bind_Closest_Bone_And_Inverse_Animation_Transform(pHand,&pBindBone);
	//Point = m_pWarpper->Warp_Snap_To_Surface(Point,pBindBone);
	//Point = m_pWarpper->Warp_Animation_Transform(Point);

	//auto Hp = m_pWarpper->Cursor_Transform(pHand , &Snaped);
	//auto Point = m_pWarpper->Animation_Transform(Hp);
	//pBindBone = m_pWarpper->Dictionary()[Hp.Indices[0]];
	//if (Hp.Indices[0] == 0)
	//	std::cout<<"Snapped invalid!"<<Hp.Indices[0]<<' '<<Hp.Indices[1]<<' '<<Hp.Indices[2]<<' '<<Hp.Indices[3]<<' '<<std::endl;
	if (m_pTarget)
		Snaped = m_pTarget->Volume().Contains(pHand);
	else
		Snaped = false;

	if (Snaped) 
		m_pPreview->Color() = SnapedIndicator;
	else
		m_pPreview->Color() = UnSnapedIndicator;
	m_pPreview->Position() = pHand;
	//m_pPreview->pBindBone = pBindBone;
}

FreeCursor::FreeCursor(ID3D11DeviceContext *pContext,const std::shared_ptr<Geometrics::DynamicMetaBallModel> & pTarget,const ICamera *pCamera, SphereLightData* pLight)
	: m_pPreview(new Preview(pContext,pCamera,pLight))
	, m_pTarget(pTarget)
{
	m_pPreview->Radius() = Editing_Tools_Default_Radius;
	m_pPreview->Color() = UnSnapedIndicator;
	Snaped = false;
	//m_pWarpper = pWarpper;
}

void FreeCursor::Start()
{
	m_pPreview->Color() = SnapedIndicator;
}

void FreeCursor::Finish()
{
	m_pPreview->Color() = DirectX::Colors::Transparent;
}

void FreeCursor::Abort()
{
	m_pPreview->Color() = DirectX::Colors::Transparent;
}

FreeCursor::~FreeCursor(void)
{
}

float FreeCursor::Radius() const
{
	return m_pPreview->Radius();
}

bool FreeCursor::TurnRadius( float deltaRadius )
{
	m_pPreview->Radius() += deltaRadius;
	return true;
}

bool FreeCursor::IsActive() const
{
	return true;
}
