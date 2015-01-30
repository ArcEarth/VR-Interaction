#include "stdafx.h"
#include "AvatarSkinEffect.h"
#include "BaseEffect.h"
#include "SkinMesh.h"
#include "Lights.h"
#include "Carmera.h"

using namespace DirectX;

#ifdef _DEBUG
const static char NormalVertexShaderFile[]				=	"..\\Debug\\VS_Skinning.cso";
const static char NormalPixelShaderFile[]					=	"..\\Debug\\PS_ColorTexture_PixelLights.cso";
const static char IndicatorVertexShaderFile[]		=	"..\\Debug\\VS_AttachIndicate.cso";
const static char IndicatorPixelShaderFile[]		=	"..\\Debug\\PS_VertexColor.cso";
#else
const static char NormalVertexShaderFile[]				=	"..\\Release\\VS_Skinning.cso";
const static char NormalPixelShaderFile[]					=	"..\\Release\\PS_ColorTexture_PixelLights.cso";
const static char IndicatorVertexShaderFile[]		=	"..\\Release\\VS_AttachIndicate.cso";
const static char IndicatorPixelShaderFile[]		=	"..\\Release\\PS_VertexColor.cso";
#endif

typedef CustomEffect<VertexPositionNormalTextureWeights> CustomEffect_PNTW;

class AvatarSkinnedEffect::Impl
{
public:
	Impl(ID3D11Device *pDevice);
	~Impl();

public:
	LightsBuffer						lights;
private:
	std::unique_ptr<CustomEffect_PNTW>	m_pBasicEffect;
	std::unique_ptr<CustomEffect_PNTW>	m_pIndicatorEffect;
	IEffect*							pCurrentEffect;
};

AvatarSkinnedEffect::Impl::Impl(ID3D11Device *pDevice)
	: m_pBasicEffect	(new CustomEffect_PNTW(pDevice,NormalVertexShaderFile,nullptr,NormalPixelShaderFile))
	, m_pIndicatorEffect(new CustomEffect_PNTW(pDevice,IndicatorVertexShaderFile,nullptr,IndicatorPixelShaderFile))
	, lights(pDevice)
{
}

AvatarSkinnedEffect::Impl::~Impl()
{
}

void EnableIndicatorView(bool value)
{
}

void XM_CALLCONV AvatarSkinnedEffect::SetWorld(FXMMATRIX value)
{}
void XM_CALLCONV AvatarSkinnedEffect::SetView(FXMMATRIX value)
{
	XMMATRIX invViewMatrix = XMMatrixInverse(nullptr, value);
	XMStoreFloat3(&m_pImpl->lights.EyePosition, invViewMatrix.r[3]);
}
void XM_CALLCONV AvatarSkinnedEffect::SetProjection(FXMMATRIX value)
{

}

void AvatarSkinnedEffect::SetPerPixelLighting(bool value)
{
}

void XM_CALLCONV AvatarSkinnedEffect::SetAmbientLightColor(FXMVECTOR value)
{}

void AvatarSkinnedEffect::SetLightEnabled(int whichLight, bool value)
{}

void XM_CALLCONV AvatarSkinnedEffect::SetLightDirection(int whichLight, FXMVECTOR value)
{}

void XM_CALLCONV AvatarSkinnedEffect::SetLightDiffuseColor(int whichLight, FXMVECTOR value)
{}

void XM_CALLCONV AvatarSkinnedEffect::SetLightSpecularColor(int whichLight, FXMVECTOR value)
{}

void AvatarSkinnedEffect::EnableDefaultLighting()
{}
