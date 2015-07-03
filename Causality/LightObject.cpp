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

Causality::Light::Light()
{
}

void Causality::Light::BeginFrame(RenderContext& context)
{
	m_DepthMap.Clear(context);
	auto pEffect = static_cast<ShadowMapGenerationEffect*>(m_pShadowMapGenerator.get());
	pEffect->SetShadowMap(m_DepthMap);
}

IEffect * Light::GetRenderEffect()
{
	return m_pShadowMapGenerator.get();
}

bool Light::AcceptRenderFlags(RenderFlags flags)
{
	return flags.Contains(RenderFlags::OpaqueObjects);
}

void Causality::Light::DisableDropShadow()
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
}

void Causality::Light::SetShadowMapBuffer(const DepthStencilBuffer & depthBuffer)
{
	m_DepthMap = depthBuffer;
}


// Inherited via IRenderable

RenderFlags Light::GetRenderFlags() const { return RenderFlags::Lights; }

bool Light::IsVisible(const BoundingFrustum & viewFrustum) const
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
