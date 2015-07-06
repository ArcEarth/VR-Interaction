#pragma once
#include <DirectXMath.h>
#include <d3d11_1.h>
#include <memory>
#include "Textures.h"

namespace DirectX
{
	class IPostProcessingEffect
	{
	public :
		virtual ~IPostProcessingEffect() {}
		virtual void SetSource(_In_ UINT numSource, _In_reads_opt_(numSource) ID3D11ShaderResourceView* const* pSources) = 0;
		virtual void Render(ID3D11DeviceContext *pContext, RenderTarget* output) = 0;
	};

	class GuassianBlurEffect : public IPostProcessingEffect
	{
		GuassianBlurEffect(ID3D11Device* pDevice);
		~GuassianBlurEffect();

		// aka. kernal radius, use strength = 1 to represent a 3x3 kernal
		void SetBlurStrength(float strength);
		void SetIntermediateBuffer(RenderTargetTexture2D& texture);

	private:
		class Impl;
		std::shared_ptr<Impl> pImpl;
	};


}