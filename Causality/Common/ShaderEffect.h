#pragma once
#if defined(_XBOX_ONE) && defined(_TITLE)
#include <d3d11_x.h>
#else
#include <d3d11_1.h>
#endif
#include <Effects.h>
namespace DirectX
{
	class IShader abstract
	{
	public:
		virtual void Apply(ID3D11DeviceContext* pContext) = 0;
		virtual void SetResource(ID3D11ShaderResourceView* pResource) = 0;
		virtual void SetSampler() = 0;
	};

	class IVertexShader abstract : public IShader
	{
	};
}
