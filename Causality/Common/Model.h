#pragma once

#include "DirectXHelper.h"
#include <vector>
#include <string>
#include "Renderable.h"
#include <Model.h>
namespace DirectX
{
	namespace Scene
	{
		template <typename _VertexType>
		class Mesh : public IRenderable
		{
		public:
			typedef _VertexType VertexType;

			static int PrimitiveCount();

		public:
			Mesh(void);
			Mesh(ID3D11Device *pDevice);
			Mesh(int VertexBufferCapablity, int IndexBufferCapablity);
			Mesh(ID3D11Device *pDevice, int VertexBufferCapablity, int IndexBufferCapablity);
			~Mesh(void);

			int VertexCount() const{
				return m_Verteices.size();
			}
			int IndexCount() const{
				return m_Indices.size();
			}

			const std::vector<VertexType>& Vertices() const{
				return m_Verteices;
			}
			const std::vector<UINT>& Indices() const{
				return m_Indices;
			}

			std::vector<VertexType>& Vertices(){
				m_IsVertexBufferDirty = true;
				return m_Verteices;
			}

			std::vector<UINT>& Indices(){
				m_IsIndexBufferDirty = true;
				return m_Indices;
			}

			// Refresh the vertices and indices buffer
			void Update(ID3D11Device *pDevice);
			// Render the model to device
			void Render(ID3D11DeviceContext *pContext);
			// Clear the inside data
			void Clear()
			{
				m_Verteices.clear();
				m_Indices.clear();
			}

		private:

			void UpdateVertexBuffer(ID3D11Device *pDevice){
				if (m_IsVertexBufferDirty){
					DirectX::SafeRelease(m_pVertexBuffer);
					if (m_Verteices.size() > 0)
						m_pVertexBuffer = DirectX::CreateVertexBuffer<VertexType>(pDevice, m_Verteices.size(), &m_Verteices[0]);
					else
						m_pVertexBuffer = nullptr;
				}
				m_IsVertexBufferDirty = false;
			}

			void UpdateIndexBuffer(ID3D11Device *pDevice){
				if (m_IsIndexBufferDirty){
					DirectX::SafeRelease(m_pIndexBuffer);
					if (m_Indices.size() > 0)
						m_pIndexBuffer = DirectX::CreateIndexBuffer(pDevice, m_Indices.size(), &m_Indices[0]);
					else
						m_pIndexBuffer = nullptr;
				}
				m_IsIndexBufferDirty = false;
			}

		protected:
			static D3D11_PRIMITIVE_TOPOLOGY s_PrimitiveType;

		private:
			Microsoft::WRL::ComPtr<ID3D11Buffer> m_pVertexBuffer;
			Microsoft::WRL::ComPtr<ID3D11Buffer> m_pIndexBuffer;
		};

		template <typename VertexType>
		Mesh<VertexType>::Mesh(void)
		{
			//#ifdef _DEBUG
			//	if (!s_IsInitialized) throw;
			//#endif // _DEBUG
			m_IsVertexBufferDirty = m_IsIndexBufferDirty = m_IsConstantBufferDirty = false;
			m_pConstantBuffer = m_pIndexBuffer = m_pVertexBuffer = nullptr;
		}

		template <typename VertexType>
		Mesh<VertexType>::Mesh(ID3D11Device *pDevice)
		{
			//#ifdef _DEBUG
			//	if (!s_IsInitialized) throw;
			//#endif // _DEBUG
			m_IsVertexBufferDirty = m_IsIndexBufferDirty = m_IsConstantBufferDirty = false;
			m_pConstantBuffer = m_pIndexBuffer = m_pVertexBuffer = nullptr;
			m_pConstantBuffer = DirectX::CreateConstantBuffer<ConstantType>(pDevice);
		}

		template <typename VertexType>
		Mesh<VertexType>::Mesh(int VertexBufferCapablity, int IndexBufferCapablity)
			: m_Verteices(VertexBufferCapablity), m_Indices(IndexBufferCapablity)
		{
			//#ifdef _DEBUG
			//	if (!s_IsInitialized) throw;
			//#endif // _DEBUG
			m_IsVertexBufferDirty = m_IsIndexBufferDirty = m_IsConstantBufferDirty = false;
			m_pConstantBuffer = m_pIndexBuffer = m_pVertexBuffer = nullptr;
		}

		template <typename VertexType>
		Mesh<VertexType>::~Mesh(void)
		{
			DirectX::SafeRelease(m_pConstantBuffer);
			DirectX::SafeRelease(m_pIndexBuffer);
			DirectX::SafeRelease(m_pVertexBuffer);
		}

		template <typename VertexType>
		int Mesh<VertexType>::PrimitiveCount(){
			int vertiesPerPrimive = 1;
			switch (s_PrimitiveType)
			{
			case D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST:
			case D3D11_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP:
				vertiesPerPrimive = 3;
				break;
			case D3D11_PRIMITIVE_TOPOLOGY_LINELIST:
			case D3D11_PRIMITIVE_TOPOLOGY_LINESTRIP:
				vertiesPerPrimive = 2;
				break;
			case D3D11_PRIMITIVE_TOPOLOGY_POINTLIST:
				vertiesPerPrimive = 1;
				break;
			default:
				break;
			}
			return m_Indices.size() / vertiesPerPrimive;
		}

		template <typename VertexType>
		void Mesh<VertexType>::Render(ID3D11DeviceContext *pContext)
		{
			if (m_IsIndexBufferDirty || m_IsVertexBufferDirty || m_pConstantBuffer == nullptr)
			{
				Microsoft::WRL::ComPtr<ID3D11Device> pDevice;
				pContext->GetDevice(&pDevice);
				Refresh(pDevice.Get());
				//pDevice->Release();
			}

			UpdateConstantBuffer(pContext);

			// Set the input assembler stage
			unsigned int stride = sizeof(VertexType);
			unsigned int offset = 0;
			pContext->IASetVertexBuffers(0, 1, &m_pVertexBuffer, &stride, &offset);
			pContext->IASetIndexBuffer(m_pIndexBuffer, DXGI_FORMAT_R32_UINT, 0);
			pContext->IASetPrimitiveTopology(s_PrimitiveType);

			// 12 is the maximum constant buffer number for a shader stage
			if (s_ConstantSlot <= 12)
				pContext->VSSetConstantBuffers(s_ConstantSlot, 1, &m_pConstantBuffer);
			pContext->DrawIndexed(m_Indices.size(), 0, 0);
			return;
		}

		template <typename VertexType>
		void Mesh<VertexType>::Update(ID3D11Device *pDevice)
		{
			UpdateVertexBuffer(pDevice);
			UpdateIndexBuffer(pDevice);
		}
	}
}
#endif // !DX_BASE_MODEL_H
