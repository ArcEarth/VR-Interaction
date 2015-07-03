#pragma once
#include "CameraObject.h"
#include <Textures.h>

namespace Causality
{
	// class to introduce a parallel or point (perspective) light
	class Light : public Camera, public IRenderable
	{
	public:
		Light();
		Light(RenderDevice& deivce, const UINT& shadowResolution = 1024U);

		// ICameraRenderControl
		virtual void		BeginFrame(RenderContext& context) override;
		DirectX::IEffect*	GetRenderEffect() override;
		virtual bool		AcceptRenderFlags(RenderFlags flags) override;

		Color	GetColor() const { return m_Color; }
		void	SetColor(const Color& color) { m_Color = color; }

		// Brightness is the among of color showed when the surface is UNIT distant with the light
		float	GetBrightness() const { return m_Color.w; }
		void	SetBrightness(float bright) { m_Color.w = bright; }

		void	DisableDropShadow();
		void	EnableDropShadow(RenderDevice& deivce, const UINT& shadowResolution = 1024U);
		void	SetShadowMapBuffer(const DirectX::DepthStencilBuffer& depthBuffer);
		ID3D11ShaderResourceView* GetShadowMap() const { return m_DepthMap.ShaderResourceView(); }

		void	SetColorMap(ID3D11ShaderResourceView* lightMap) { m_pColorMap = lightMap; }
		ID3D11ShaderResourceView* GetColorMap() const {
			return m_pColorMap.Get();
		}

		bool	DropShadow() const;
	private:
		DirectX::Color						m_Color;
		cptr<ID3D11ShaderResourceView>		m_pColorMap;
		DirectX::DepthStencilBuffer		    m_DepthMap;
		sptr<DirectX::IEffect>				m_pShadowMapGenerator;

		// Inherited via IRenderable
		virtual RenderFlags GetRenderFlags() const;
		virtual bool IsVisible(const BoundingFrustum & viewFrustum) const override;
		virtual void Render(RenderContext & context, DirectX::IEffect* pEffect = nullptr) override;
		virtual void XM_CALLCONV UpdateViewMatrix(DirectX::FXMMATRIX view, DirectX::CXMMATRIX projection) override;
	};

	class ParallelLight : public Light
	{

	};

	class SpotLight : public Light
	{

	};

	class SphereLight : public Light
	{

	};
}
