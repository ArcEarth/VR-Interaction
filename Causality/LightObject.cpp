#include "pch_bcl.h"
#include "CameraObject.h"
#include <DirectXColors.h>
#include "LightObject.h"
#include <PrimitiveVisualizer.h>
#include <ShadowMapGenerationEffect.h>

using namespace DirectX;
using namespace Causality;

std::weak_ptr<ShadowMapGenerationEffect> g_wpSMGEffect;


Light::Light(RenderDevice & device, const UINT& shadowResolution)
{
	EnableDropShadow(device, shadowResolution);
}

Light::Light()
{
}

void Light::Begin(RenderContext& context)
{
	m_RenderTarget.Clear(context,m_Background);
	auto pEffect = static_cast<ShadowMapGenerationEffect*>(m_pShadowMapGenerator.get());

	m_RenderTarget.SetAsRenderTarget(context);

	if (pEffect)
	{
		pEffect->SetShadowMap(NULL, NULL);
		pEffect->SetView(GetViewMatrix());
		pEffect->SetProjection(GetProjectionMatrix());
	}
}

void Light::End()
{
}

IEffect * Light::GetRenderEffect()
{
	return m_pShadowMapGenerator.get();
}

bool Light::AcceptRenderFlags(RenderFlags flags)
{
	bool result = flags.Contains(RenderFlags::OpaqueObjects);
	return result;
}

Color Causality::Light::GetColor() const { return m_Color; }

void Light::DisableDropShadow()
{
	m_pShadowMapGenerator.reset();
	m_RenderTarget.Reset();
}

void Light::EnableDropShadow(RenderDevice & device, const UINT & resolution)
{
	if (!g_wpSMGEffect.expired())
		m_pShadowMapGenerator = g_wpSMGEffect.lock();
	else
	{
		auto pEffect = std::make_shared<ShadowMapGenerationEffect>(device.Get());
		g_wpSMGEffect = pEffect;
		m_pShadowMapGenerator = pEffect;
	}

	CD3D11_VIEWPORT viewport(.0f, .0f, resolution, resolution);
	m_RenderTarget = RenderTarget(
		RenderableTexture2D(device.Get(), resolution, resolution, DXGI_FORMAT_R16_UNORM),
		DepthStencilBuffer(device.Get(), resolution, resolution, DXGI_FORMAT_D16_UNORM),
		viewport);
}

void Light::SetShadowMapBuffer(const DepthStencilBuffer & depthBuffer)
{
	m_RenderTarget.DepthBuffer() = depthBuffer;
}

ID3D11ShaderResourceView * Causality::Light::GetShadowMap() const { 
	return m_RenderTarget.DepthBuffer().ShaderResourceView();
	//return m_RenderTargetTex.ShaderResourceView();
}

ID3D11ShaderResourceView * Causality::Light::GetColorMap() const {
	return m_pColorMap.Get();
}


// Inherited via IRenderable

RenderFlags Light::GetRenderFlags() const { return RenderFlags::Lights; }

bool Light::IsVisible(const BoundingGeometry & viewFrustum) const
{
	return false;
}

void Light::Render(RenderContext & context, DirectX::IEffect* pEffect)
{
}

void DrawFrustum(BoundingFrustum& frustum)
{
}

void XM_CALLCONV Light::UpdateViewMatrix(FXMMATRIX view, CXMMATRIX projection)
{
	auto &frustum = GetViewFrustum();
	auto& drawer = Visualizers::g_PrimitiveDrawer;
}

DirectX::XMVECTOR Causality::ILight::GetFocusDirection() const
{
	using namespace DirectX;
	XMMATRIX view = GetViewMatrix();
	view = XMMatrixTranspose(view);
	return XMVectorSelect(g_XMSelect1110.v, -view.r[2], g_XMSelect1110.v);
}
