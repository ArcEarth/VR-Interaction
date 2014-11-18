#pragma once

#include "DirectXHelper.h"
#include "DXGIFormatHelper.h"
#include <vector>
#include <string>
#include "Renderable.h"
#include <Effects.h>
#include <type_traits>
#include <VertexTypes.h>
#include <DirectXCollision.h>
namespace DirectX
{
	namespace Scene
	{
		//namespace IndexPrimitives
		//{
		//	template <class _TIndex>
		//	struct Triangle
		//	{
		//		union
		//		{
		//			_TIndex[3] V;
		//			struct
		//			{
		//				_TIndex V0, V1, V2;
		//			}
		//		};
		//		inline _TIndex& operator[](size_t idx)
		//		{
		//			return V[idx];
		//		}
		//		inline _TIndex operator[](size_t idx) const
		//		{
		//			return V[idx];
		//		}
		//	};
		//}
		// A Container of Vertex and Indices with Given IEffect to render
		// Should be use with std::shared_ptr
		class Mesh : public IRenderable
		{
		public:
			template<class _TVertex,class _TIndex>
			static std::shared_ptr<Mesh> Create(ID3D11Device *pDevice, const std::vector<_TVertex>& vertices, const std::vector<_TIndex> indices,const std::shared_ptr<IEffect>& pEffect = nullptr,D3D_PRIMITIVE_TOPOLOGY primitiveTopology = D3D_PRIMITIVE_TOPOLOGY::D3D10_PRIMITIVE_TOPOLOGY_TRIANGLELIST)
			{
				return Create(pDevice, &vertices[0], vertices.size(), &indices[0], indices.size(), pEffect, primitiveTopology);
			}

			template<class _TVertex, class _TIndex>
			static std::shared_ptr<Mesh> Create(ID3D11Device* pDevice, const _TVertex* vertices, unsigned int VerticesCount, const _TIndex* indices, unsigned int IndicesCount, const std::shared_ptr<IEffect>& pEffect = nullptr, D3D_PRIMITIVE_TOPOLOGY primitiveTopology = D3D_PRIMITIVE_TOPOLOGY::D3D10_PRIMITIVE_TOPOLOGY_TRIANGLELIST,size_t VertexStride = sizeof(_TVertex), UINT startIndex = 0, UINT VertexOffset = 0)
			{
				static_assert(std::is_integral<_TIndex>::value,"Type of Index must be integral type");
				static_assert(_TVertex::InputElementCount, "Valiad Vertex Type should have static InputElements/InputElementCount member");
				assert(pDevice != nullptr);
				auto pMesh = std::make_shared<Mesh>();


				if (pEffect != nullptr)
				{
					void const* shaderByteCode;
					size_t byteCodeLength;
					pEffect->GetVertexShaderBytecode(&shaderByteCode, &byteCodeLength);
					ThrowIfFailed(
						pDevice->CreateInputLayout(_TVertex::InputElements, _TVertex::InputElementCount, shaderByteCode, byteCodeLength, &pMesh->pInputLayout)
					);
				}

				pMesh->pVertexBuffer = DirectX::CreateVertexBuffer<_TVertex>(pDevice, VerticesCount, vertices);
				pMesh->pIndexBuffer = DirectX::CreateIndexBuffer(pDevice, IndicesCount, indices);
				pMesh->IndexFormat = ExtractDXGIFormat<_TIndex>::value;
				pMesh->VertexCount = VerticesCount;
				pMesh->IndexCount = IndicesCount;
				pMesh->StartIndex = 0;
				pMesh->VertexStride = sizeof(_TVertex);
				pMesh->VertexOffset = 0;
				pMesh->pEffect = pEffect;
				pMesh->PrimitiveType = primitiveTopology;

				return pMesh;
			}

			uint32_t                                                IndexCount;
			uint32_t												VertexCount;
			uint32_t                                                StartIndex;
			uint32_t                                                VertexOffset;
			uint32_t                                                VertexStride;
			D3D_PRIMITIVE_TOPOLOGY                                  PrimitiveType;
			DXGI_FORMAT                                             IndexFormat;
			Microsoft::WRL::ComPtr<ID3D11InputLayout>               pInputLayout;
			Microsoft::WRL::ComPtr<ID3D11Buffer>                    pIndexBuffer;
			Microsoft::WRL::ComPtr<ID3D11Buffer>                    pVertexBuffer;
			std::shared_ptr<IEffect>                                pEffect;
			bool                                                    IsAlpha;

			// Inherited via IRenderable
			virtual void Render(ID3D11DeviceContext *pContext) override;
		};

		template<class _TVertex, class _TIndex, D3D_PRIMITIVE_TOPOLOGY Primative_Topology = D3D_PRIMITIVE_TOPOLOGY::D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST>
		class TypedMesh : public Mesh
		{
		public:
			typedef TypedMesh SelfType;
			typedef _TVertex VertexType;
			typedef _TIndex IndexType;
			static inline std::shared_ptr<SelfType> Create(ID3D11Device *pDevice, const std::vector<_TVertex>& vertices, const std::vector<_TIndex> indices, D3D_PRIMITIVE_TOPOLOGY primitiveTopology = D3D_PRIMITIVE_TOPOLOGY::D3D10_PRIMITIVE_TOPOLOGY_TRIANGLELIST, const std::shared_ptr<IEffect>& pEffect = nullptr )
			{
				return std::static_pointer_cast<std::shared_ptr<SelfType>>(Mesh::Create<VertexType, IndexType>(pDevice,vertices,indices, Primative_Topology, pEffect));
			}
		};

		class ObjMesh : public TypedMesh<VertexPositionNormalTexture,uint16_t>
		{
		public:
			static std::shared_ptr<ObjMesh> CreateFromFile(std::wstring file);
		};

		class RigidMesh : public Mesh, public LocalCoordinate
		{

		};
	}
}

