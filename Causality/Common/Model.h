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
		namespace FacetPrimitives
		{
			template <class _TIndex>
			struct Triangle
			{
				union
				{
					_TIndex[3] V;
					struct
					{
						_TIndex V0, V1, V2;
					}
				};

				inline _TIndex& operator[](size_t idx)
				{
					return V[idx];
				}

				inline _TIndex operator[](size_t idx) const
				{
					return V[idx];
				}

				operator XMUINT3() const
				{
					return XMUINT3(V0, V1, V2);
				}
			};
		}
		// A Container of Vertex and Indices with Given IEffect to render
		// Should be use with std::shared_ptr
		struct Mesh : public IRenderable
		{
		public:
			template<class _TVertex, class _TIndex>
			void Update(ID3D11Device* pDevice, const _TVertex* vertices, unsigned int VerticesCount, const _TIndex* indices, unsigned int IndicesCount, const std::shared_ptr<IEffect>& pEffect = nullptr, D3D_PRIMITIVE_TOPOLOGY primitiveTopology = D3D_PRIMITIVE_TOPOLOGY::D3D10_PRIMITIVE_TOPOLOGY_TRIANGLELIST,size_t VertexStride = sizeof(_TVertex), UINT startIndex = 0, UINT VertexOffset = 0)
			{
				static_assert(std::is_integral<_TIndex>::value,"Type of Index must be integral type");
				static_assert(_TVertex::InputElementCount, "Valiad Vertex Type should have static InputElements/InputElementCount member");
				assert(pDevice != nullptr);

				if (pEffect != nullptr)
				{
					void const* shaderByteCode;
					size_t byteCodeLength;
					pEffect->GetVertexShaderBytecode(&shaderByteCode, &byteCodeLength);
					ThrowIfFailed(
						pDevice->CreateInputLayout(_TVertex::InputElements, _TVertex::InputElementCount, shaderByteCode, byteCodeLength, &pMesh->pInputLayout)
					);
				}

				pVertexBuffer = DirectX::CreateVertexBuffer<_TVertex>(pDevice, VerticesCount, vertices);
				pIndexBuffer = DirectX::CreateIndexBuffer(pDevice, IndicesCount, indices);
				IndexFormat = ExtractDXGIFormat<_TIndex>::value;
				VertexCount = VerticesCount;
				IndexCount = IndicesCount;
				StartIndex = 0;
				VertexStride = sizeof(_TVertex);
				VertexOffset = 0;
				pEffect = pEffect;
				PrimitiveType = primitiveTopology;
			}

			template<class _TVertex, class _TIndex>
			void Update(ID3D11Device *pDevice, const std::vector<_TVertex>& vertices, const std::vector<_TIndex> indices, const std::shared_ptr<IEffect>& pEffect = nullptr, D3D_PRIMITIVE_TOPOLOGY primitiveTopology = D3D_PRIMITIVE_TOPOLOGY::D3D10_PRIMITIVE_TOPOLOGY_TRIANGLELIST)
			{
				Update(pDevice, &vertices[0], vertices.size(), &indices[0], indices.size(), pEffect, primitiveTopology);
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
		struct TypedMesh : public Mesh
		{
		public:
			typedef TypedMesh SelfType;
			typedef _TVertex VertexType;
			typedef _TIndex IndexType;

			void Update(ID3D11Device *pDevice, const std::vector<_TVertex>& vertices, const std::vector<_TIndex> indices, D3D_PRIMITIVE_TOPOLOGY primitiveTopology = D3D_PRIMITIVE_TOPOLOGY::D3D10_PRIMITIVE_TOPOLOGY_TRIANGLELIST, const std::shared_ptr<IEffect>& pEffect = nullptr )
			{
				Mesh::Update<VertexType, IndexType>(pDevice,vertices,indices, Primative_Topology, pEffect);
			}
		};

		// A Mesh that also keeps the vertices and facets information in CPU
		template<class _TVertex = VertexPositionNormalTexture, class _TIndex = uint32_t>
		struct GeometryMesh : public TypedMesh<_TVertex, _TIndex> , public LocalCoordinate
		{
		public:
			typedef GeometryMesh SelfType;
			GeometryMesh() {}
			GeometryMesh(std::wstring file);
			void Update(ID3D11Device *pDevice,const std::shared_ptr<IEffect>& pEffect = nullptr)
			{
				TypedMesh::Update(pDevice,Vertices,Facets)
			}
		public:
			std::vector<VertexType>			Vertices;
			std::vector<DirectX::XMUINT3>	Facets;
		};

		template<class _TVertex, class _TIndex>
		GeometryMesh<_TVertex, _TIndex>::GeometryMesh(std::wstring file)
		{
			std::ifstream stream;
			stream.open(file);

			if (!stream.is_open())
				throw;

			string tag;
			Vector3 vec3;
			Vector2 vec2;
			Color color;
			XMUINT3 tri;

			std::vector<Vector3> positions;
			std::vector<Vector3> normals;
			std::vector<Vector2> texcoords;

			color.w = 1.0f;
			while (!stream.eof())
			{
				stream >> tag;

				if (tag == "v") // (x,y,z[,w]) coordinates, w is optional and defaults to 1.0.
				{
					stream >> vec3.x >> vec3.y >> vec3.z;
					positions.push_back(vec3);
				}
				else if (tag == "vt") // in (u, v [,w]), these will vary between 0 and 1, w is optional and defaults to 0.
				{
					stream >> vec2.x >> vec2.y;
					texcoords.push_back(vec2);
				}
				else if (tag == "vn") // (u, v [,w]) normals might not be unit. 
				{
					stream >> vec3.x >> vec3.y >> vec3.z;
					normals.push_back(vec3);
				}
				else if (tag == "vp") // ( u [,v] [,w] ) free form geometry statement
				{
					stream >> vec3.x >> vec3.y >> vec3.z; //Ignore the unknow parameter data
														  //VertexParameters.push_back(vec3);
				}
				else if (tag == "f")
				{
					stream >> tri.x >> tri.y >> tri.z;	// f v1 v2 v3 ...
														// f v1/vt1/vn1 v2/vt2/vn2 v3/vt3/vn3 ...
														// f v1//vn1 v2//vn2 v3//vn3 ...
					tri.x--, tri.y--, tri.z--; // The obj file format starts the indices from 1, but as usual, it should starts from 0
					Facets.push_back(tri);
				}
			}

			auto N = positions.size();
			assert(normals.size() == 0 || N == normals.size());
			assert(texcoords.size() == 0 || N == texcoords.size());

			BoundingBox::CreateFromPoints(Bounds, postions.size(), &postions[0], sizeof(Vector3));
			Vertices.resize(N);

			if (!HasVertexNormal())
				GenerateVertexNormal();

			stride_range<Vector3> vpos(&Vertices[0].Position, sizeof(VertexType), N);
			std::copy_n(positions.begin(), N, vpos.begin());
			stride_range<Vector3> vnor(&Vertices[0].Normal, sizeof(VertexType), N);
			std::copy_n(normals.begin(), N, vnor.begin());
			stride_range<Vector3> vtex(&Vertices[0].TexCoord, sizeof(VertexType), N);
			std::copy_n(texcoords.begin(), N, vtex.begin());
		}

		class RigidMesh : public Mesh
		{

		};
	}
}

