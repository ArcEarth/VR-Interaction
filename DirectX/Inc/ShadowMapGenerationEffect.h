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
		void SetShadowMap(ID3D11DepthStencilView* pShaodwMap);

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