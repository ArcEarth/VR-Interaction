#pragma once
#include <d3d11_1.h>
#include <DirectXMath.h>
//#include <utility>

namespace DirectX{

	class IRenderObject{
	public:
		virtual void Render(ID3D11DeviceContext *pContext) = 0;
		virtual ~IRenderObject(){};
//		virtual void Refresh(ID3D11Device *pDevice) = 0;
	};

	//class IGeometricsObject{
	//public:
	//	virtual void Tessellate(unsigned int VertexCount , XMFLOAT3* Positions , XMFLOAT3* Normals , unsigned int IndexCount , unsigned int Indecis*);
	//};

	class ITextureObject
	{
	public:
		virtual DirectX::XMFLOAT2 UVMapping(FXMVECTOR VertexPosition) const = 0;
	};

	class ISkinObject
	{
	public:
		virtual void Weighting(DirectX::FXMVECTOR PosVtr,_Out_writes_(4) float* BlendWeights ,_Out_writes_(4) UINT32 * BlendIndices) const = 0;
	};

}