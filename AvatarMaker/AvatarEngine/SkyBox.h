#pragma once
#include "modelinterface.h"
#include <VertexTypes.h>
#include <wrl\client.h>
#include <string>
#include <Effects.h>
#include "DirectXHelper.h"
#include "DXGIFormatHelper.h"
#include "Carmera.h"

namespace DirectX
{
	template <class _TVertex , UINT Stride = sizeof(_TVertex)>
	class VertexBuffer
	{
	public:
		typedef _TVertex VertexType;

		VertexBuffer(ID3D11Device *pDevice ,int Capablity ,const VertexType* pInitialData ,UINT CPUAccessFlag = 0)
		{
			CD3D11_BUFFER_DESC VertexBufferDesc(sizeof(VertexType)*Capablity, D3D11_BIND_VERTEX_BUFFER, D3D11_USAGE_DEFAULT, CPUAccessFlag);
			D3D11_SUBRESOURCE_DATA * pInitialSubresource = nullptr;
			if (pInitialData){
				pInitialSubresource = new D3D11_SUBRESOURCE_DATA();
				pInitialSubresource->pSysMem = pInitialData;
				pInitialSubresource->SysMemPitch = 0;
				pInitialSubresource->SysMemSlicePitch = 0;
			}
			ThrowIfFailed(
				pDevice->CreateBuffer(
					&VertexBufferDesc,
					pInitialSubresource,
					&m_pBuffer
					)
				);
			m_Offset = 0;
		}

		operator ID3D11Buffer* const *()
		{
			return m_pBuffer.GetAddressOf():
		}

		VertexBuffer(VertexBuffer& rhs , UINT offset)
		{
			m_pBuffer = rhs.m_pBuffer;
			Offset = offset;
		}

		unsigned int							Offset;

	protected:
		unsigned int							m_Size;
		unsigned int							m_Capablity;
		Microsoft::WRL::ComPtr<ID3D11Buffer>	m_pBuffer;
	};

	template <class _TVertex , class _TIndex = uint16_t , D3D11_PRIMITIVE_TOPOLOGY Primative_Topology = D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST>
	class BasicMesh
		: public IRenderObject
	{
	public:
		typedef _TVertex VertexType;
		typedef _TIndex IndexType;
		const static D3D11_PRIMITIVE_TOPOLOGY PrimitiveType = Primative_Topology;
	public:
		BasicMesh()
		{
			m_pVertexBuffer = nullptr;
			m_pIndexBuffer = nullptr;
			m_VerticesCount = 0;
			m_IndicesCount = 0;
		}

		BasicMesh(ID3D11Device* pDevice,const _TVertex* vertices , unsigned int VerticesCount ,const IndexType* indices , unsigned int IndicesCount)
		{
			Initialize(pDevice,vertices,VerticesCount,indices,IndicesCount);
		}

		void Initialize(ID3D11Device* pDevice,const _TVertex* vertices , unsigned int VerticesCount ,const IndexType* indices , unsigned int IndicesCount)
		{
			m_pVertexBuffer = DirectX::CreateVertexBuffer<VertexType>(pDevice,VerticesCount,vertices);
			m_pIndexBuffer = DirectX::CreateIndexBuffer(pDevice,IndicesCount,indices);
			m_VerticesCount = VerticesCount;
			m_IndicesCount = IndicesCount;
		}

		virtual void Render(ID3D11DeviceContext* pDeviceContext)
		{
			// Set vertex buffer stride and offset.
			unsigned int stride = sizeof(VertexType);
			unsigned int offset = 0;
			auto pBuffer = m_pVertexBuffer.Get();
			pDeviceContext->IASetVertexBuffers(0,1,&pBuffer,&stride,&offset);
			pDeviceContext->IASetIndexBuffer(m_pIndexBuffer.Get(),ExtractDXGIFormat<IndexType>::value,0);
			pDeviceContext->IASetPrimitiveTopology(PrimitiveType);
			pDeviceContext->DrawIndexed(m_IndicesCount,0,0);
		}

	protected:
		Microsoft::WRL::ComPtr<ID3D11Buffer>		m_pVertexBuffer;
		Microsoft::WRL::ComPtr<ID3D11Buffer>		m_pIndexBuffer;
		unsigned int								m_VerticesCount;
		unsigned int								m_IndicesCount;
	};

	class CubeTexture;

	class SkyBox :
		public BasicMesh<VertexPositionTexture,uint16_t,D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST>
	{
		const static unsigned int VerticesCount = 24;
		const static unsigned int IndicesCount = 36;
		const static VertexType CubeVertices[VerticesCount];
		const static IndexType CubeIndices[IndicesCount];
	public:
		SkyBox(ID3D11Device* pDevice , const ICamera* pCamera , const std::wstring (&TextureFiles)[6]);

		virtual void Render(ID3D11DeviceContext* pDeviceContext);

		~SkyBox(void);

	protected:
		const ICamera*								m_pCamera;
		std::unique_ptr<BasicEffect>				m_pEffect;
		std::unique_ptr<CubeTexture>				m_pCubeTexture;
		Microsoft::WRL::ComPtr<ID3D11InputLayout>	m_pInputLayout;
	};

	class FloorPlane
		: public BasicMesh<VertexPositionNormalTexture,uint16_t,D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST>
	{
	public:
		const static unsigned int VerticesCount = 4;
		const static unsigned int IndicesCount = 6;
		const static VertexType Vertices[VerticesCount];
		const static IndexType Indices[IndicesCount];

		FloorPlane(ID3D11Device* pDevice , const ICamera* pCamera , const std::wstring &TextureFile);
		~FloorPlane(void);

		void SetFloorTexture(ID3D11ShaderResourceView* pTextureSRV);
		void SetFloorPlaneEquation(DirectX::FXMVECTOR PlaneEquation);

		virtual void Render(ID3D11DeviceContext* pDeviceContext);

	protected:
		const ICamera*										m_pCamera;
		std::unique_ptr<BasicEffect>						m_pEffect;
		Microsoft::WRL::ComPtr<ID3D11Resource>				m_pTexture;
		Microsoft::WRL::ComPtr<ID3D11ShaderResourceView>	m_pTextureView;
		Microsoft::WRL::ComPtr<ID3D11SamplerState>			m_pSamplerState;
		Microsoft::WRL::ComPtr<ID3D11InputLayout>			m_pInputLayout;		
	};
}

