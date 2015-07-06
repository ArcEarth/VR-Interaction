#pragma once
#include "ShaderEffect.h"

namespace DirectX
{
	class ShadowMapGenerationEffect
		: public IEffect, public IEffectMatrices, public IEffectSkinning
	{
	public:
		ShadowMapGenerationEffect(ID3D11Device* device);
		~ShadowMapGenerationEffect();

		// Shadow Map setting
		enum ShadowFillMode
		{
			DepthFill = 0,
			SolidColorFill = 1,
		};

		void SetShadowMap(ID3D11DepthStencilView* pShaodwMap, ID3D11RenderTargetView* pRTV = NULL);
		void XM_CALLCONV SetShadowFillMode(_In_ ShadowFillMode mode, _In_opt_ FXMVECTOR color = Colors::Black);

		// Use texture for alpha clipping
		void SetAlphaDiscardThreshold(float clipThreshold);
		void SetAlphaDiscardTexture(ID3D11ShaderResourceView* pTexture);
		void DisableAlphaDiscard();

		// IEffectMatrices
		virtual void XM_CALLCONV SetWorld(FXMMATRIX value) override;
		virtual void XM_CALLCONV SetView(FXMMATRIX value) override;
		virtual void XM_CALLCONV SetProjection(FXMMATRIX value) override;

		// IEffectSkinning
		static const size_t MaxBones = 72;
		virtual void __cdecl SetWeightsPerVertex(int value) override;
		virtual void __cdecl SetBoneTransforms(_In_reads_(count) XMMATRIX const* value, size_t count) override;
		virtual void __cdecl ResetBoneTransforms() override;

		// Inherited via IEffect
		virtual void Apply(ID3D11DeviceContext * deviceContext) override;
		virtual void GetVertexShaderBytecode(void const ** pShaderByteCode, size_t * pByteCodeLength) override;
	private:
		class Impl;
		std::unique_ptr<Impl> pImpl;
	};

}