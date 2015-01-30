#include "stdafx.h"
#include "ColorPalette.h"
#include <GeometricPrimitive.h>
#include "RectangleDrawer.h"

using namespace DirectX;
using namespace DirectX::Colors;
using namespace std;

using namespace EditingTools::PaintTools;

static RectDrawer s_RectDrawer;

void DrawRect(ID3D11DeviceContext* pContext ,  BoundingBox* pBox,FXMVECTOR Color)
{
//	s_RectDrawer.DrawRect_Pixel(pContext,100,100,200,200,Color);
	s_RectDrawer.DrawRect_ProjectionCoor_CenterExtent(pContext,pBox->Center.x,pBox->Center.y,pBox->Extents.x,pBox->Extents.y,Color);
}

static XMFLOAT3A SamplingPoints[5];


ColorPalette::ColorPalette(ID3D11Device* pDevice ,const ICamera* pCamera)
	: m_pCamera(pCamera)
{
	s_RectDrawer.Initialize(pDevice);
	Initialize();
}

ColorPalette::ColorPalette(const ICamera* pCamera)
	: m_pCamera(pCamera)
{
	Initialize();
}

void ColorPalette::Initialize()
{
	m_Colors[0] = RoyalBlue;
	m_Colors[1] = ForestGreen;
	m_Colors[2] = Black;
	m_Colors[3] = Yellow;
	m_Colors[4] = Crimson;
	m_Colors[5] = White;

	// This is all the coordinate in Projection Space
	float Lx = 0.15f , Ly = 0.2f;
	float x=0.5f , y=-0.25f;
	for (int i=0;i<3;i++){
		m_PigmentBoxes[i].Center = XMFLOAT3(x , y , 0);
		m_PigmentBoxes[i].Extents = XMFLOAT3(Lx,Ly, 100.0f);
		y += 0.5f;
	}
	x=-0.5f , y=-0.25f;
	for (int i=3;i<6;i++){
		m_PigmentBoxes[i].Center = XMFLOAT3(x , y , 0);
		m_PigmentBoxes[i].Extents = XMFLOAT3(Lx,Ly, 100.0);
		y += 0.5f;
	}
}

XMVECTOR ColorPalette::GetColor(FXMVECTOR Point) const{
	XMMATRIX ViewMatrix = m_pCamera->GetViewMatrix();
	XMMATRIX ProjectionMatrix = m_pCamera->GetProjectionMatrix();
	XMMATRIX ViewProjectionMatrix = ViewMatrix * ProjectionMatrix;
	// Get the position in Projection Space
	XMVECTOR Pr = Point;

	XMVECTOR vPos = XMVector3TransformCoord(Point,ViewProjectionMatrix);

	XMStoreFloat3A(&SamplingPoints[0],vPos);

	//for (int i = 1; i < 5; i++)
	//{
	//	Pr += 0.5f * g_XMIdentityR2;
	//	XMVECTOR vProj = XMVector3TransformCoord(Pr,ViewProjectionMatrix);
	//	XMStoreFloat3A(&SamplingPoints[i],vProj);
	//}


	for (int i = 0; i < 6; i++)
	{
		if (m_PigmentBoxes[i].Contains(vPos) != DISJOINT)
			return m_Colors[i];
	}
	return Transparent;
}


ColorPalette::~ColorPalette(void)
{
}

XMVECTOR ColorPalette::GetColor(unsigned index) const {
	return m_Colors[index];
}

void ColorPalette::Render(ID3D11DeviceContext* pContext)
{
	//for (int i = 0; i < 5; i++)
	//{
	//	s_RectDrawer.DrawRect_ProjectionCoor_CenterExtent(pContext,SamplingPoints[i].x,SamplingPoints[i].y,0.02f,0.02f,m_Colors[i]);
	//}
	XMVECTOR BackGround = XMVectorSet(1.0f,1.0f,1.0f,1.0f);
	for (int i = 0; i < 6; i++)
	{
		XMVECTOR Color = XMVectorSet(m_Colors[i].x,m_Colors[i].y,m_Colors[i].z,1.0f);
		m_PigmentBoxes[i].Extents.x *= 1.05f;
		m_PigmentBoxes[i].Extents.y *= 1.05f;
		DrawRect(pContext,&m_PigmentBoxes[i],BackGround);
		m_PigmentBoxes[i].Extents.x /= 1.05f;
		m_PigmentBoxes[i].Extents.y /= 1.05f;
		DrawRect(pContext,&m_PigmentBoxes[i],Color);
	}
}
