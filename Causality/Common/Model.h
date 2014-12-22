#pragma once
#include "stride_iterator.h"
#include "DirectXHelper.h"
#include "DXGIFormatHelper.h"
#include <vector>
#include <string>
#include "Renderable.h"
#include <Effects.h>
#include <type_traits>
#include <VertexTypes.h>
#include <DirectXCollision.h>
#include "Textures.h"
#include "Material.h"

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
					_TIndex V[3];
					struct
					{
						_TIndex V0, V1, V2;
					};
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

		class IMesh abstract
		{
		public:
			virtual ~IMesh() {}
			// Setup Vertex/Index Buffer and Call the Draw command
			virtual void Draw(ID3D11DeviceContext* pContext) const = 0;
		};

		// A Container of Vertex and Indices holding geometry information with Identical effect to render
		// Should be use with std::shared_ptr
		struct Mesh : public IMesh
		{
		public:
			~Mesh(){}

			template<class _TVertex, class _TIndex>
			void CreateDeviceResources(ID3D11Device* pDevice, const _TVertex* vertices, unsigned int VerticesCount, const _TIndex* indices, unsigned int IndicesCount, D3D_PRIMITIVE_TOPOLOGY primitiveTopology = D3D_PRIMITIVE_TOPOLOGY::D3D10_PRIMITIVE_TOPOLOGY_TRIANGLELIST, size_t VertexStride = sizeof(_TVertex), UINT startIndex = 0, UINT VertexOffset = 0 , const std::shared_ptr<IEffect> &pEffect = nullptr)
			{
				static_assert(std::is_integral<_TIndex>::value, "Type of Index must be integral type");
				static_assert(_TVertex::InputElementCount, "Valiad Vertex Type should have static InputElements/InputElementCount member");
				assert(pDevice != nullptr);

				if (pEffect != nullptr)
				{
					void const* shaderByteCode;
					size_t byteCodeLength;
					pEffect->GetVertexShaderBytecode(&shaderByteCode, &byteCodeLength);
					ThrowIfFailed(
						pDevice->CreateInputLayout(_TVertex::InputElements, _TVertex::InputElementCount, shaderByteCode, byteCodeLength, &pInputLayout)
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
				PrimitiveType = primitiveTopology;
			}

			typedef std::vector<std::unique_ptr<Mesh>> Collection;


			uint32_t                                                IndexCount;
			uint32_t												VertexCount;
			uint32_t                                                StartIndex;
			uint32_t                                                VertexOffset;
			uint32_t                                                VertexStride;
			D3D_PRIMITIVE_TOPOLOGY                                  PrimitiveType;
			DXGI_FORMAT                                             IndexFormat;
			InputDescription										pInputDescription;
			Microsoft::WRL::ComPtr<ID3D11InputLayout>               pInputLayout;
			Microsoft::WRL::ComPtr<ID3D11Buffer>                    pIndexBuffer;
			Microsoft::WRL::ComPtr<ID3D11Buffer>                    pVertexBuffer;
			bool                                                    IsAlpha;

			// Setup the Vertex/Index Buffer and call the draw command
			void Draw(ID3D11DeviceContext *pContext) const;
		};

		// MeshPart with static vertex typeing
		template<class _TVertex, class _TIndex, D3D_PRIMITIVE_TOPOLOGY Primative_Topology = D3D_PRIMITIVE_TOPOLOGY::D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST>
		struct TypedMesh : public Mesh
		{
		public:
			static_assert(std::is_integral<_TIndex>::value, "IndexType is not integer");

			// Setup the effect for rendering and change the input layout as well
			void SetEffect(const std::shared_ptr<IEffect> &pRenderEffect)
			{
				pEffect = pRenderEffect;
			}

			typedef TypedMesh	SelfType;
			typedef _TVertex	VertexType;
			typedef _TIndex		IndexType;
		};

		//class IModel
		//{
		//public:
		//	virtual void Render(ID3D11DeviceContext *pContext, IEffect* pEffect) = 0;
		//};

		// A ModelPart is a aggregate of a mesh and it's Material
		class ModelPart
		{ 
		public:
			std::string	Name;
			std::shared_ptr<Mesh>			pMesh;
			std::shared_ptr<PhongMaterial>	pMaterial;
			DirectX::BoundingBox			BoundBox;
			DirectX::BoundingOrientedBox	BoundOrientedBox;
			void Render(ID3D11DeviceContext *pContext, IEffect* pEffect);
		};

		class LocalMatrixHolder : virtual public ILocalMatrix
		{
		public:
			// Inherited via ILocalMatrix
			void XM_CALLCONV SetModelMatrix(DirectX::FXMMATRIX model)
			{
				XMStoreFloat4x4(&LocalMatrix, model);
			}

			XMMATRIX GetModelMatrix() const
			{
				return XMLoadFloat4x4(&LocalMatrix);
			}
			Matrix4x4	LocalMatrix;
		};

		class IModelNode : virtual public ILocalMatrix, virtual public IBoundable
		{
		public:
			virtual ~IModelNode();

			virtual void Render(ID3D11DeviceContext *pContext, IEffect* pEffect) = 0;

			// Get the Recursive multiplied model matrix since root, return the global world matrix
			virtual XMMATRIX GetWorldMatrix() const;
			//virtual XMMATRIX GetModelMatrix() const override;
			// Transformed OrientedBounding Box
			virtual BoundingOrientedBox GetOrientedBoundingBox() const override;
			// Transformed Bounding Box
			virtual BoundingBox GetBoundingBox() const override;
			// Transformed Bounding Sphere
			virtual BoundingSphere GetBoundingSphere() const override;

			virtual void XM_CALLCONV SetModelMatrix(DirectX::FXMMATRIX model) override;

			virtual XMMATRIX GetModelMatrix() const override;


			// Inherited via ILocalMatrix
			//virtual void XM_CALLCONV SetModelMatrix(DirectX::FXMMATRIX model) override;
			//virtual XMMATRIX GetModelMatrix() const override;

			IModelNode*			pParent = nullptr;
			std::string			Name;
			BoundingOrientedBox BoundOrientedBox;
			BoundingSphere		BoundSphere;
			BoundingBox			BoundBox;
			Matrix4x4			LocalMatrix;
			float				Opticity;
		};

		// A basic model is a collection of ModelPart shares same Local Matrix
		// Leaf node in a model tree
		class BasicModel : public IModelNode
		{
		public:
			std::vector<std::shared_ptr<ModelPart>>	Parts;

			virtual void Render(ID3D11DeviceContext *pContext, IEffect* pEffect) override;
		};

		// Inherit from vector type, only push_back is valiad !!!
		class ModelCollection : public IModelNode, public std::vector<std::shared_ptr<IModelNode>>
		{
		public:
			typedef std::vector<std::shared_ptr<IModelNode>> ContainnerType;
			// All the data in Children's postion/orientation is 
			// In the local coordinate of it's parent!
			using ContainnerType::operator[];
			void push_back(const value_type& _Val);
			void push_back(value_type&& _Val);
			virtual void Render(ID3D11DeviceContext *pContext, IEffect* pEffect) override;
		};

		// This Model also keeps the geomreics data in CPU
		class GeometryModel : public BasicModel
		{
		public:
			static bool CreateFromObjFile(GeometryModel *pResult, ID3D11Device *pDevice, const std::wstring &file, const std::wstring& textureDir);
			BasicModel* ReleaseCpuResource();
		public:
			std::vector<VertexPositionNormalTexture>			Vertices;
			std::vector<FacetPrimitives::Triangle<uint16_t>>	Facets;
			stride_range<Vector3>	Positions;
			stride_range<Vector3>	Normals;
			stride_range<Vector2>	TexCoords;
		};

	}
}

