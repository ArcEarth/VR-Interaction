#pragma once
#include "dxbasemodel.h"
#include <DirectXMath.h>
#include "MathHelper.h"
#include "Textures.h"
#include "BitMap.h"
#include <VertexTypes.h>

const unsigned int VERTEX_BLEND_BONES = 4U;
const unsigned int MAX_BLEND_MATRIX_COUNT = 255U;

namespace DirectX{
	struct SkinObjectBuffer
	{
		DirectX::XMFLOAT4X4 WorldMatrix;
		DirectX::XMFLOAT3X4 BlendMatrices[MAX_BLEND_MATRIX_COUNT];
	};
	struct DualQuaternionBonesBuffer
	{
		DirectX::XMFLOAT4X4 WorldMatrix;
		DirectX::SimpleMath::DualQuaternion BlendMatrices[MAX_BLEND_MATRIX_COUNT];
	};

	struct VertexPositionNormalTextureWeights
	{
		DirectX::XMFLOAT4 Position;
		DirectX::XMFLOAT3 Normal;
		DirectX::XMFLOAT2 TexCoord;
		DirectX::XMUINT4  Indices;
		DirectX::XMFLOAT4 Weights;
		static const UINT InputElementCount = 5;
		static const D3D11_INPUT_ELEMENT_DESC InputElements[InputElementCount];
	};

	typedef VertexPositionNormalTextureWeights SkinVertex;

	typedef	DirectX::BaseModel<SkinVertex,SkinObjectBuffer> SkinMesh;

	class TextureSkinMesh 
		: public SkinMesh
	{
	public:
		static_assert (VERTEX_BLEND_BONES<=4U,"Vertex blend bone count must <= 4");
		static const unsigned int Default_Texture_Resolution = 1024;
		static UINT s_TextureSlot;

		TextureSkinMesh();
		TextureSkinMesh(ID3D11Device *pDevice , unsigned int Resolution = Default_Texture_Resolution);
		TextureSkinMesh(ID3D11Device *pDevice , const std::shared_ptr<Texture2D>& pTexture);
	// Interface methods
	public:
		virtual void Render(ID3D11DeviceContext* pContext);
		//virtual void Refresh(ID3D11Device* pDevice);
		virtual ~TextureSkinMesh();

	// Feature method
	public:
		//Bitmap& Texture_W();
		//const Bitmap& Texture() const;
		Texture2D*			Texture();
		const Texture2D*	Texture() const;
		XMUINT2 TextureSize() const;

		void SetTexture(const std::shared_ptr<Texture2D>& pTexture);

	private:
		//void UpdateTextureBuufer(ID3D11DeviceContext* pContext);

	private:
		bool m_IsTextureBufferDirty;
		//Bitmap m_TextureBuffer;
		ID3D11SamplerState *m_pSampler;
		std::shared_ptr<Texture2D> m_pTexture;
	};
}