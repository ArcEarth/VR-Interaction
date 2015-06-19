#pragma once
#include "stride_range.h"
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
#include "ConstantBuffer.h"

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

		class IMeshBuffer abstract
		{
		public:
			virtual ~IMeshBuffer() {}
			// Setup Vertex/Index Buffer and Call the Draw command
			virtual void Draw(ID3D11DeviceContext* pContext) const = 0;
		};

		// A Container of Vertex and Indices holding geometry information with Identical effect to render
		// Abstraction of mesh's vertex information on GPU
		// Should be use with std::shared_ptr
		// Holding the information of 
		// Vertex Buffer, Index Buffer, Input Layout 
		// The abstraction for IA and Draw commands
		struct MeshBuffer : public IMeshBuffer
		{
		public:
			~MeshBuffer(){}

			template<class _TVertex, class _TIndex>
			void CreateDeviceResources(ID3D11Device* pDevice, const _TVertex* vertices, unsigned int VerticesCount, const _TIndex* indices, unsigned int IndicesCount, IEffect *pEffect = nullptr, D3D_PRIMITIVE_TOPOLOGY primitiveTopology = D3D_PRIMITIVE_TOPOLOGY::D3D10_PRIMITIVE_TOPOLOGY_TRIANGLELIST, size_t VertexStride = sizeof(_TVertex), UINT startIndex = 0, UINT VertexOffset = 0 )
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

			typedef std::vector<std::unique_ptr<MeshBuffer>> Collection;

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

			// Setup the Vertex/Index Buffer and call the draw command
			void Draw(ID3D11DeviceContext *pContext) const;
		};

		namespace GeometricPrimtives
		{
			std::shared_ptr<MeshBuffer> CreateCube();
		}

		// Strong typed mesh buffer with static vertex typeing
		template<class _TVertex, class _TIndex, D3D_PRIMITIVE_TOPOLOGY Primative_Topology = D3D_PRIMITIVE_TOPOLOGY::D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST>
		struct TypedMeshBuffer : public MeshBuffer
		{
		public:
			static_assert(std::is_integral<_TIndex>::value, "IndexType is not integer");

			typedef TypedMeshBuffer	SelfType;
			typedef _TVertex	VertexType;
			typedef _TIndex		IndexType;
		};

		//class IModel
		//{
		//public:
		//	virtual void Render(ID3D11DeviceContext *pContext, IEffect* pEffect) = 0;
		//};

		// A ModelPart is a aggregate of a mesh and it's Material
		struct ModelPart
		{ 
		public:
			std::string						Name;
			std::shared_ptr<MeshBuffer>		pMesh;		// Mesh of this model part
			std::shared_ptr<PhongMaterial>	pMaterial;	// Material of this model part
			std::shared_ptr<IEffect>		pEffect;	// Default Effect of this model part
			DirectX::BoundingBox			BoundBox;
			DirectX::BoundingOrientedBox	BoundOrientedBox;

			// This render method will not set "Transform"
			void Render(ID3D11DeviceContext *pContext, IEffect* pEffect = nullptr);
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

		class IModelNode : virtual public IBoundable
		{
		public:
			virtual ~IModelNode();

			// Property : Name
			virtual const std::string& Name() const = 0;
			virtual void SetName(const std::string& name) = 0;

			// Method : Render
			virtual void Render(ID3D11DeviceContext *pContext, const Matrix4x4& transform, IEffect* pEffect = nullptr) = 0;

			// Get the Recursive multiplied model matrix since root, return the global world matrix
			// virtual XMMATRIX GetWorldMatrix() const;
			//virtual XMMATRIX GetModelMatrix() const override;
			// Transformed OrientedBounding Box
			//virtual BoundingOrientedBox GetOrientedBoundingBox() const = 0;
			// Transformed Bounding Box
			//virtual BoundingBox GetBoundingBox() const override;
			// Transformed Bounding Sphere
			// virtual BoundingSphere GetBoundingSphere() const;

			//virtual void XM_CALLCONV SetModelMatrix(DirectX::FXMMATRIX model) override;

			//virtual XMMATRIX GetModelMatrix() const override;


			// Inherited via ILocalMatrix
			//virtual void XM_CALLCONV SetModelMatrix(DirectX::FXMMATRIX model) override;
			//virtual XMMATRIX GetModelMatrix() const override;

			#pragma region PropertyParent
		//	IModelNode* Parent() { return pParent; }
		//	const IModelNode* Parent() const { return pParent; }
		//	void SetParent(IModelNode* parent) { pParent = parent; }

		//private:
		//	IModelNode*			pParent = nullptr;
			#pragma endregion

			//BoundingOrientedBox BoundOrientedBox;
			//BoundingSphere		BoundSphere;
			//BoundingBox			BoundBox;
			//Matrix4x4			LocalMatrix;
			//float				Opticity;
		};

		// A Monolith Model is a model with single ModelPart
		class MonolithModel : public IModelNode, public ModelPart
		{
		public:
			virtual void Render(ID3D11DeviceContext *pContext, const Matrix4x4& transform, IEffect* pEffect) override;

			const std::string& Name() const override { return ModelPart::Name; }
			void SetName(const std::string& name) { ModelPart::Name = name; }

			// Inherited via IModelNode
			virtual BoundingBox GetBoundingBox() const override;
			virtual BoundingOrientedBox GetOrientedBoundingBox() const override;

		};

		// A basic model is a collection of ModelPart shares same Local Matrix
		// Leaf node in a model tree
		class CompositionModel : public IModelNode
		{
		protected:
			std::string				_Name;
		public:
			BoundingOrientedBox		BoundOrientedBox;
			BoundingSphere			BoundSphere;
			BoundingBox				BoundBox;

			std::vector<ModelPart>	Parts;

			virtual void Render(ID3D11DeviceContext *pContext, const Matrix4x4& transform, IEffect* pEffect) override;
			const std::string& Name() const override { return _Name; }
			void SetName(const std::string& name) { _Name = name; }

			// Inherited via IModelNode
			virtual BoundingBox GetBoundingBox() const override;
			virtual BoundingOrientedBox GetOrientedBoundingBox() const override;
		};

		// Inherit from vector type, only push_back is valiad !!!
		class CollectionModel : public IModelNode
		{
		public:
			struct ModelTransformPair
			{
				MatrixTransform	Transform;
				std::shared_ptr<IModelNode> Model;
			};

			typedef std::vector<ModelTransformPair> ContainnerType;

		protected:
			std::string				_Name;
		public:
			ContainnerType			Children;
		public:
			//BoundingOrientedBox		BoundOrientedBox;
			BoundingBox				BoundBox;

		public:

			// All the data in Children's postion/orientation is 
			// In the local coordinate of it's parent!
			void AddChild(const std::shared_ptr<IModelNode> &model, const MatrixTransform &transform = reinterpret_cast<const MatrixTransform&>(MatrixTransform::Identity));
			//void push_back(const value_type& _Val);
			//void push_back(value_type&& _Val);
			virtual void Render(ID3D11DeviceContext *pContext, const Matrix4x4& transform, IEffect* pEffect) override;

			const std::string& Name() const { return _Name; }
			void SetName(const std::string& name) { _Name = name; }

			// Inherited via IModelNode
			virtual BoundingBox GetBoundingBox() const override;
			//virtual BoundingOrientedBox GetOrientedBoundingBox() const override;

		};

		// This Model also keeps the geomreics data in CPU
		class GeometryModel : public CompositionModel
		{
		public:
			typedef VertexPositionNormalTexture VertexType;
			typedef uint16_t					IndexType;

			static bool CreateFromObjFile(GeometryModel *pResult, ID3D11Device *pDevice, const std::wstring &file, const std::wstring& textureDir);
			void CreateDeviceResource(ID3D11Device *pDevice);
			CompositionModel* ReleaseCpuResource();

		public:
			std::vector<VertexPositionNormalTexture>			Vertices;
			std::vector<FacetPrimitives::Triangle<uint16_t>>	Facets;
			stdx::stride_range<Vector3>							Positions;
			stdx::stride_range<Vector3>							Normals;
			stdx::stride_range<Vector2>							TexCoords;
		};

	}
}

