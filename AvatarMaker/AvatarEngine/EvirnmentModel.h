#pragma once
#include "ModelInterface.h"
#include <VertexTypes.h>

#include "dxbasemodel.h"
#include "BitMap.h"
#include "Textures.h"

struct BaseObjectBuffer
{
	DirectX::XMFLOAT4X4 WorldMatrix;
};

namespace DirectX
{
	template <typename VertexType,typename ConstantType> 
	class DynamicTextureMesh
		: public DirectX::BaseModel<VertexType,ConstantType>
	{
	public:
		DynamicTextureMesh(void);
		~DynamicTextureMesh(void);

		Bitmap& Texture_W();
		const Bitmap& Texture() const;
		XMUINT2 TextureSize() const;
		virtual void Render(ID3D11DeviceContext* pContext);

	private:
		void UpdateTextureBuufer(ID3D11DeviceContext* pContext);

	private:
		bool m_IsTextureBufferDirty;
		Bitmap m_TextureBuffer;
		ID3D11SamplerState *m_pSampler;
		DynamicTexture2D *m_pTexture;
	};



}