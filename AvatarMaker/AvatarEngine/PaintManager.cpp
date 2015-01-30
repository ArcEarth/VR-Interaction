#include "stdafx.h"
#include "PaintManager.h"
#include <GeometricPrimitive.h>

using namespace DirectX;
using namespace EditingTools::PaintTools;
using namespace Geometrics;

//GaussBrush::GaussBrush(unsigned int Radius)
//{
//	_radius = Radius;
//}
//
//bool GaussBrush::Draw(Bitmap* Canvas , XMFLOAT2 Coordinate , FXMVECTOR Color)
//{
//	if (!Canvas) return false;
//	unsigned x=(unsigned)(Coordinate.x*(Canvas->Width()-1)),y=(unsigned)(Coordinate.y*(Canvas->Height()-1));
//	unsigned Xmin = max(x-Radius,0),Xmax = min(x+Radius,Canvas->Width());
//	unsigned Ymin = max(y-Radius,0),Ymax = min(y+Radius,Canvas->Height());
//
//	for (unsigned int i = Xmin; i < Xmax; i++)
//	{
//		for (unsigned int j = Ymin; j < Ymax; j++)
//		{
//			float r2 = (float)((i-x)*(i-x)+(j-y)*(j-y));
//			float alpha = expf(-5*r2/(Radius*Radius));
//			//XMVECTOR pixel = XMLoadFloat4A(&Canvas->at(i,j));
//			XMVECTOR pixel = Canvas->GetPixel(i,j);
//			pixel *= (1-alpha);
//			pixel += Color*alpha;
//			Canvas->SetPixel(i,j,pixel);
//			//XMStoreFloat4A(&Canvas->at(i,j),pixel);
//		}
//	}
//	return true;
//}
//
//bool PaintBucket::Draw(Bitmap* Canvas , XMFLOAT2 Coordinate , FXMVECTOR Color)
//{
//	if (!Canvas) return false;
//	//XMFLOAT4A color;
//	//XMStoreFloat4A(&color,Color);
//	//Canvas->Dye(color);
//	Canvas->Dye(Color);
//	return true;
//}

ColorHoverPicker::ColorHoverPicker(FXMVECTOR InitialColor /* = Colors::White*/)
	: m_PaintColor(InitialColor)
{
}


bool ColorHoverPicker::HoverColor(FXMVECTOR Color)
{
	if (XMVector4Equal(Color,Colors::Transparent)) 
	{
		//m_Timer.Reset();
		return false;
	}
	XMVECTOR UpdatingColor = XMLoadFloat4(&m_UpdatingColor);
	if (!XMVector4Equal(Color,UpdatingColor)){
		m_Timer.Reset();
		m_UpdatingColor = Color;
		return false;
	}else
	{
		if (m_Timer.GetTime() > Hover_Select_Time)
		{
			m_PaintColor = m_UpdatingColor;
			m_Timer.Reset();
			return true;
		}else
		{
			return false;
		}
	}
}

bool ColorHoverPicker::HoverPick(FXMVECTOR Position)
{
	auto PlletteColor = s_pPalette->GetColor(Position);
	return this->HoverColor(PlletteColor);
}

void ColorHoverPicker::SetColor(FXMVECTOR Color)
{
	m_PaintColor = Color;
	m_Timer.Reset();
	return ;
}

std::unique_ptr<ColorPalette> ColorHoverPicker::InitializeColorPalette(ID3D11Device* pDevice,const ICamera* pCamera)
{
	s_pPalette = new ColorPalette(pDevice,pCamera);
	return std::unique_ptr<ColorPalette>(s_pPalette);
}

//void ColorHoverPicker::ReleaseColorPalette()
//{
// delete s_pPalette;
//}

ColorPalette* ColorHoverPicker::GetColorPalette()
{
	return s_pPalette;
}



class EditingTools::PaintTools::GeometricObject
	: public IRenderObject
{
public:
	GeometricObject(ID3D11DeviceContext *pContext ,const ICamera *pCamera)
		: m_pCamera(pCamera)
	{
		m_pPrimitve = GeometricPrimitive::CreateSphere(pContext);
	}

	void Render(ID3D11DeviceContext *pContext){
		XMMATRIX World = XMMatrixScaling(Radius,Radius,Radius) * XMMatrixTranslationFromVector((XMVECTOR)Position);
		XMMATRIX View = m_pCamera->GetViewMatrix();
		XMMATRIX Projection = m_pCamera->GetProjectionMatrix();

		m_pPrimitve->Draw(World,View,Projection,(XMVECTOR)Color);
	}
public:
	float Radius;
	Vector4 Color;
	Vector4 Position;
protected:
	const ICamera *m_pCamera;
	std::unique_ptr<GeometricPrimitive> m_pPrimitve;
};

PaintToolBase::PaintToolBase(const std::shared_ptr<Geometrics::DynamicMetaBallModel> &pSubject , const std::shared_ptr<IColorSource> &pSource , ID3D11DeviceContext* pContext ,const ICamera* pCamera , SphereLightData* pCursorLight)
	: m_pSubject(pSubject)
	//, m_pTextureMapper(pTextureMapper)
	, m_pSource(pSource)
	//, m_pWarpper(pWarpper)
	, m_pCursorLight(pCursorLight)
	, m_pCamera(pCamera)
	, m_pVisual(new PaintTools::GeometricObject(pContext,pCamera))
	, m_working(false)
	, MandatorySnaped(true)
{
	SetRadius(Editing_Tools_Default_Radius);
}

PaintToolBase::~PaintToolBase()
{
}

void PaintToolBase::Start(){
	if (IsActive()) return;
	// Backup the texture
	m_working = true;
}

void PaintToolBase::Abort()
{
	if (!IsActive()) return;
	// Recover the texture to the state before start
	//m_pSubject->Texture_W() = *m_pTexBackup;
	m_working = false;	
}

void PaintToolBase::Finish(){
	if (!IsActive()) return;
	m_working = false;	
}

IRenderObject* PaintToolBase::Visualization()
{
	if (!m_Snaped)
		return	m_pVisual.get();
	else return nullptr;
}

float PaintToolBase::Radius() const
{
	return m_radius;
}

bool PaintToolBase::SetRadius(float Radius){
	m_radius = Radius;
	if (m_radius<0.03f) {
		m_radius = 0.03f;
		return false;
	}
	if (m_radius>0.5f) {
		m_radius = 0.5f;
		return false;
	}
	m_pVisual->Radius = m_radius * Metabll_Radius_Display_Factor;
	return true;
}

bool PaintToolBase::TurnRadius(float deltaR){
	return SetRadius(m_radius += deltaR);
}

void PaintToolBase::Edit(const Vector3& pHand,const Vector3&,const Vector3&,const Vector3&)
{
	if (!IsActive()) return;

	m_Snaped = false;
	float SnapedRadiu = 0.15f;
	if (MandatorySnaped)
		SnapedRadiu = 1.0f;

	XMVECTOR vOrigin = m_pCamera->GetPosition();
	XMVECTOR vDirection = pHand - vOrigin;
	Vector3 intersectionPoint = pHand;
	m_pSubject->AnimationUpdate();
	const auto& model = m_pSubject->Volume();
	//m_Snaped = model.FindRayIntersectionPointWithMesh(intersectionPoint,vOrigin,vDirection);
	m_Snaped = model.RayIntersection(intersectionPoint,vOrigin,vDirection);

	XMVECTOR vPos = intersectionPoint;

	Edit(vPos,m_Snaped);

	//auto Hp = m_pWarpper->Cursor_Transform(pHand,&m_Snaped,SnapedRadiu);

	//Edit(Hp , m_Snaped);

	//auto vPos = m_pWarpper->Animation_Transform(Hp);

	if (m_Snaped && m_pCursorLight)
	{
		m_pCursorLight->Color = m_pSource->GetColor();
		m_pCursorLight->Position = vPos;
		m_pVisual->Color = m_pSource->GetColor();
		m_pVisual->Color.w = 0.5f;
		m_pVisual->Position = vPos;
	} else
	{
		//std::cout<<"Paint Not Snapped!"<<std::endl;
		if (m_pCursorLight)
			m_pCursorLight->Color = DirectX::Colors::Transparent;
		m_pVisual->Color = m_pSource->GetColor();
		m_pVisual->Color.w = 0.5f;
		m_pVisual->Position = vPos;
	}
}

bool PaintToolBase::IsActive() const{
	return m_working;
}

BrushTool::BrushTool(const std::shared_ptr<Geometrics::DynamicMetaBallModel> &pSubject , const std::shared_ptr<IColorSource> &pSource , ID3D11DeviceContext* pContext ,const ICamera* pCamera , SphereLightData* pCursorLight)
	: PaintToolBase(pSubject,pSource,pContext,pCamera,pCursorLight)
	//, m_pTexBackup(new Bitmap(pSubject->TextureSize().x,pSubject->TextureSize().y))
{
	MandatorySnaped = false;
}

bool BrushTool::SetRadius(float Radius)
{
	auto hr = PaintToolBase::SetRadius(Radius);
	m_pBrush->Radius = PaintToolBase::Radius();
	return hr;
}

void BrushTool::Start()
{
	if (!m_pBrush)
		throw std::logic_error("Use before initialize.");
	m_pBrush->Start();
	PaintToolBase::Start();
	//*m_pTexBackup = m_pSubject->Texture();
}

void BrushTool::Finish()
{
	m_pBrush->Finish();
	PaintToolBase::Finish();
}



void BrushTool::Abort()
{
	m_pBrush->Finish();
	PaintToolBase::Abort();
	//m_pSubject->Texture_W() = *m_pTexBackup;
}

//void BrushTool::Edit(const HolographicPoint& hPosDS , bool Snaped)
//{
//	//if (Snapped)
//	//{
//		XMVECTOR vPosWS = m_pWarpper->Animation_Transform(hPosDS);
//		auto pTarget = dynamic_cast<RenderTargetTexture2D*>(m_pSubject->Skin.Texture());
//		m_pBrush->Draw(m_pContext,pTarget,vPosWS,m_pSource->GetColor());
//		//auto &canvas = m_pSubject->Texture_W();
//		//m_pBrush->Draw(&canvas,m_pTextureMapper->UVMapping(Ptr.Position),m_pSource->GetColor());
//	//}
//}

void EyedropperTool::Start()
{
	PaintToolBase::Start();
	m_ColorBackup = m_pSource->GetColor();
}

void EyedropperTool::Abort()
{
	PaintToolBase::Abort();
	m_pSource->SetColor((XMVECTOR)m_ColorBackup);
}

//void EyedropperTool::Edit(const HolographicPoint& Ptr , bool Snaped)
//{
//	XMVECTOR CursorPos = m_pWarpper->Animation_Transform(Ptr);
//	XMVECTOR vColor = m_pColorPallete->GetColor(CursorPos);
//	m_pSource->SetColor(vColor);
//	//if (Snaped)
//	//{
//	//	auto Dp = Ptr.Position;
//	//	//auto colorT = m_pSubject->Texture().at(m_pTextureMapper->UVMapping(Dp));
//	//	//auto color = XMLoadFloat4A(&colorT);
//	//	//m_pSource->SetColor(color);
//	//	auto Coor = m_pTextureMapper->UVMapping(Dp);
//	//	XMVECTOR Color = m_pSource->GetColor();
//	//	//auto Color = m_pSubject->Texture().GetPixel(Coor.x,Coor.y);
//	//	m_pSource->SetColor(Color);
//	//}
//}

void EyedropperTool::Edit(DirectX::FXMVECTOR vPos , bool Snaped)
{
	XMVECTOR vColor = m_pColorPallete->GetColor(vPos);
	m_pSource->SetColor(vColor);
}

void HoverCursorTool::Edit(DirectX::FXMVECTOR vPos , bool Snaped)
{
	m_pHoverPicker->HoverPick(vPos);
}

void BrushTool::Edit(DirectX::FXMVECTOR vPos , bool Snaped)
{
	if (!Snaped) return;
	auto pTarget = dynamic_cast<RenderTargetTexture2D*>(m_pSubject->Skin().Texture());
	m_pBrush->Draw(m_pContext,pTarget,vPos,m_pSource->GetColor());
}


EyedropperTool::EyedropperTool(const std::shared_ptr<Geometrics::DynamicMetaBallModel> &pSubject , const std::shared_ptr<IColorSource> &pSource , ID3D11DeviceContext* pContext ,const ICamera* pCamera , const std::shared_ptr<ColorPalette>& pPalette)
	: PaintToolBase(pSubject,pSource,pContext,pCamera,nullptr)
	, m_pColorPallete(pPalette)
{
}

HoverCursorTool::HoverCursorTool(const std::shared_ptr<Geometrics::DynamicMetaBallModel> &pSubject , const std::shared_ptr<IColorSource> &pSource , ID3D11DeviceContext* pContext ,const ICamera* pCamera , SphereLightData* pCursorLight,const std::shared_ptr<ColorHoverPicker> &pColorHoverPicker)
	: m_pHoverPicker(pColorHoverPicker)
	, PaintToolBase(pSubject,pSource,pContext,pCamera,pCursorLight)
{
	PaintToolBase::MandatorySnaped = false;
	m_pSource = m_pHoverPicker;
}

std::shared_ptr<IColorSource> HoverCursorTool::GetColorSource() const
{
	return m_pHoverPicker;
}

//
//void HoverCursorTool::Edit(const HolographicPoint& Ptr , bool Snaped)
//{
//	auto Ap = m_pWarpper->Animation_Transform(Ptr);
//	m_pHoverPicker->HoverPick(Ap);
//}


ColorPalette* ColorHoverPicker::s_pPalette;
