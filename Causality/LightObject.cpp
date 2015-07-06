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

void Light::BeginFrame(RenderContext& context)
{
	m_DepthMap.Clear(context);
	auto pEffect = static_cast<ShadowMapGenerationEffect*>(m_pShadowMapGenerator.get());
	pEffect->SetShadowMap(m_DepthMap, m_RenderTargetTex);
	CD3D11_VIEWPORT viewport(.0f,.0f,m_DepthMap.Width(), m_DepthMap.Height());
	context->RSSetViewports(1, &viewport);
}

void Light::SetView(size_t view)
{
	auto pEffect = static_cast<ShadowMapGenerationEffect*>(m_pShadowMapGenerator.get());
	if (pEffect)
	{
		pEffect->SetView(GetViewMatrix());
		pEffect->SetProjection(GetProjectionMatrix());
	}
}

void Light::EndFrame()
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
	m_DepthMap.Reset();
}

void Light::EnableDropShadow(RenderDevice & device, const UINT & shadowResolution)
{
	if (!g_wpSMGEffect.expired())
		m_pShadowMapGenerator = g_wpSMGEffect.lock();
	else
	{
		auto pEffect = std::make_shared<ShadowMapGenerationEffect>(device.Get());
		g_wpSMGEffect = pEffect;
		m_pShadowMapGenerator = pEffect;
	}
	m_DepthMap = DepthStencilBuffer(device.Get(), shadowResolution, shadowResolution, DXGI_FORMAT_D16_UNORM);
	m_RenderTargetTex = RenderTargetTexture2D(device.Get(), shadowResolution, shadowResolution,DXGI_FORMAT_R16_UNORM);
}

void Light::SetShadowMapBuffer(const DepthStencilBuffer & depthBuffer)
{
	m_DepthMap = depthBuffer;
}

ID3D11ShaderResourceView * Causality::Light::GetShadowMap() const { 
	return m_DepthMap.ShaderResourceView(); 
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
	auto &frustum = GetViewFrustum(0);
	auto& drawer = Visualizers::g_PrimitiveDrawer;
}

DirectX::XMVECTOR Causality::ILight::GetFocusDirection() const
{
	using namespace DirectX;
	XMMATRIX view = GetViewMatrix();
	view = XMMatrixTranspose(view);
	return XMVectorSelect(g_XMSelect1110.v, -view.r[2], g_XMSelect1110.v);
}
