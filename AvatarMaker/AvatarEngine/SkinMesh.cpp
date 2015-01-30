#include "stdafx.h"
#include "SkinMesh.h"

namespace DirectX{
	//static member for Vertex 
	const D3D11_INPUT_ELEMENT_DESC SkinVertex::InputElements[] = 
	{
		{ "SV_Position"	, 0, DXGI_FORMAT_R32G32B32A32_FLOAT	, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0 },
		{ "NORMAL"		, 0, DXGI_FORMAT_R32G32B32_FLOAT	, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0 },
		{ "TEXCOORD"	, 0, DXGI_FORMAT_R32G32_FLOAT		, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0 },
		{ "BLENDINDICES", 0, DXGI_FORMAT_R32G32B32A32_UINT	, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0 },
		{ "BLENDWEIGHT"	, 0, DXGI_FORMAT_R32G32B32A32_FLOAT	, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0 },
	};
	//static member for mesh
	//ID3D11VertexShader *SkinMesh::s_pVertexShader;
	//ID3D11PixelShader *SkinMesh::s_pPixelShader;
	//ID3D11GeometryShader *SkinMesh::s_pGeometryShader;
	//ID3D11InputLayout *SkinMesh::s_pInputLayout;
	//D3D11_PRIMITIVE_TOPOLOGY SkinMesh::s_PrimitiveType;
	//bool SkinMesh::s_IsInitialized;
	UINT SkinMesh::s_ConstantSlot = 0;
	D3D11_PRIMITIVE_TOPOLOGY SkinMesh::s_PrimitiveType = D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST;

	UINT TextureSkinMesh::s_TextureSlot = 0;

	TextureSkinMesh::TextureSkinMesh(ID3D11Device *pDevice , unsigned int Resolution)
		: SkinMesh(pDevice)
		//, m_TextureBuffer(Resolution,Resolution)
		, m_pTexture(std::make_shared<RenderTargetTexture2D>(pDevice,Resolution,Resolution))
		//, m_pTexture(new DynamicTexture2D(pDevice,Resolution,Resolution))
	{
		m_IsTextureBufferDirty = true;
		m_pSampler = DirectX::CreateSamplerState(pDevice);
		ZeroMemory(&ConstantBuffer_W(),sizeof(SkinMesh::ConstantType));
		for (auto& matrix : ConstantBuffer_W().BlendMatrices)
			matrix._11 = matrix._22 = matrix._33 = 1.0f;

		auto& matrix = ConstantBuffer_W().WorldMatrix;
		matrix._11 = matrix._22 = matrix._33 = matrix._44 = 1.0f;
		//m_pTexture = new DynamicTexture2D(pDevice,Resolution,Resolution);
		//m_pTexture->Initialize(pDevice,Resolution,Resolution);
	}

	TextureSkinMesh::TextureSkinMesh(ID3D11Device *pDevice , const std::shared_ptr<Texture2D>& pTexture)
		: SkinMesh(pDevice)
		//, m_TextureBuffer(Resolution,Resolution)
		, m_pTexture(pTexture)
	{
		m_IsTextureBufferDirty = true;
		m_pSampler = DirectX::CreateSamplerState(pDevice);
		ZeroMemory(&ConstantBuffer_W(),sizeof(SkinMesh::ConstantType));
		for (auto& matrix : ConstantBuffer_W().BlendMatrices)
			matrix._11 = matrix._22 = matrix._33 = 1.0f;
		auto& matrix = ConstantBuffer_W().WorldMatrix;
		matrix._11 = matrix._22 = matrix._33 = matrix._44 = 1.0f;
	}


	TextureSkinMesh::TextureSkinMesh()
		: SkinMesh()
		//, m_TextureBuffer()
	{
		m_pSampler = nullptr;
		m_pTexture = nullptr;
		m_IsTextureBufferDirty = false;
	};


	TextureSkinMesh::~TextureSkinMesh()
	{
		SafeRelease(m_pSampler);
	}

	void TextureSkinMesh::Render(ID3D11DeviceContext* pContext)
	{
		//UpdateTextureBuufer(pContext);

		auto pResourceView = m_pTexture->ShaderResourceView();
		pContext->PSSetShaderResources(s_TextureSlot,1,&pResourceView);
		pContext->PSSetSamplers(s_TextureSlot,1,&m_pSampler);

		SkinMesh::Render(pContext);
	}

	//void TextureSkinMesh::UpdateTextureBuufer(ID3D11DeviceContext* pContext)
	//{
	//	if (m_IsTextureBufferDirty) {
	//		m_pTexture->SetData(pContext,m_TextureBuffer.RawData());
	//		m_IsTextureBufferDirty = false;
	//	}
	//}

	//Bitmap& TextureSkinMesh::Texture_W()
	//{
	//	m_IsTextureBufferDirty = true;
	//	return m_TextureBuffer;
	//}

	//const Bitmap& TextureSkinMesh::Texture() const
	//{
	//	return m_TextureBuffer;
	//}
	Texture2D* TextureSkinMesh::Texture()
	{
		return m_pTexture.get();
	}

	const Texture2D* TextureSkinMesh::Texture() const
	{
		return m_pTexture.get();
	}

	void TextureSkinMesh::SetTexture(const std::shared_ptr<Texture2D>& pTexture)
	{
		m_pTexture = pTexture;
	}

	XMUINT2 TextureSkinMesh::TextureSize() const
	{
		return XMUINT2(m_pTexture->Height(),m_pTexture->Width());
		//return XMUINT2(m_TextureBuffer.Height(),m_TextureBuffer.Width());
	}

}