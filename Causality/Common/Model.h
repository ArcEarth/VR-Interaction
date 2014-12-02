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

		class IModel
		{
		public:
			virtual void Render(ID3D11DeviceContext *pContext, IEffect* pEffect) = 0;
		};

		// A ModelPart is a aggregate of a mesh and it's Material
		class ModelPart : public IModel
		{ 
		public:
			std::string	Name;
			std::shared_ptr<Mesh>			pMesh;
			std::shared_ptr<PhongMaterial>	pMaterial;
			DirectX::BoundingBox			BoundBox;
			DirectX::BoundingOrientedBox	BoundOrientedBox;
			virtual void Render(ID3D11DeviceContext *pContext, IEffect* pEffect) override;
		};

		class ModelNode : public RigidObject, public IBoundable, public IModel
		{
		public:
			virtual ~ModelNode() {}
			//virtual bool IsLeaf() const = 0;
			//virtual std::type_info Type() const = 0;
			virtual XMMATRIX GetModelMatrix() const override;

		public:
			// Transformed OrientedBounding Box
			virtual BoundingOrientedBox GetOrientedBoundingBox() const override;

			// Transformed Bounding Box
			virtual BoundingBox GetBoundingBox() const override;

			// Transformed Bounding Sphere
			virtual BoundingSphere GetBoundingSphere() const override;
			std::string			Name;
			ModelNode*			pParent = nullptr;
			BoundingOrientedBox BoundOrientedBox;
			BoundingSphere		BoundSphere;
			BoundingBox			BoundBox;
		};

		// A model is a collection of ModelPart that shares identical LocalMatrix\
		// Leap node in a model tree
		class Model : public ModelNode
		{
		public:
			std::vector<ModelPart>	Parts;

			virtual void Render(ID3D11DeviceContext *pContext, IEffect* pEffect) override;
		};

		class ModelCollection : public ModelNode
		{
		public:
			// All the data in Children's postion/orientation is 
			// In the local coordinate of it's parent!
			std::vector<std::shared_ptr<ModelNode>> Children;
			void AddModel(const std::shared_ptr<ModelNode>& pModel);
			virtual void Render(ID3D11DeviceContext *pContext, IEffect* pEffect) override;
		};

		// This Model also keeps the geomreics data in CPU
		class GeometryModel : public Model
		{
		public:
			~GeometryModel();

	/*		template <class ...TArgs>
			friend std::shared_ptr<GeometryModel> std::make_shared<GeometryModel, TArgs>(TArgs&&...);*/

			static std::shared_ptr<GeometryModel> CreateFromObjFile(ID3D11Device *pDevice, const std::wstring &file, const std::wstring& textureDir);

		public:

			GeometryModel(ID3D11Device *pDevice, const std::wstring &file, const std::wstring& textureDir);

		public:
			std::vector<VertexPositionNormalTexture>			Vertices;
			std::vector<FacetPrimitives::Triangle<uint16_t>>	Facets;
			stride_range<Vector3>	Positions;
			stride_range<Vector3>	Normals;
			stride_range<Vector2>	TexCoords;
		};

	}
}

