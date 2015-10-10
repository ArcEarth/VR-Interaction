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

		struct VertexDescription
		{
			UINT NumElements;
			const D3D11_INPUT_ELEMENT_DESC* pInputElementDescs;

			bool HasUVInfo() const;
			bool HasNormal() const;
			bool HasColor() const;
			bool HasTagent() const;
			bool HasBinormal() const;
			bool HasSkinningWeights() const;
		};

		// A Container of Vertex and Indices holding geometry information with Identical effect to render
		// Abstraction of mesh's vertex information on GPU
		// Should be use with std::shared_ptr
		// Holding the information of 
		// Vertex Buffer, Index Buffer, Input Layout 
		// The abstraction for IA and Draw commands
		struct MeshBuffer
		{
		public:
			~MeshBuffer() {}

			static ComPtr<ID3D11InputLayout>& LookupInputLayout(const D3D11_INPUT_ELEMENT_DESC* pInputElements, IEffect * pEffect);

			template<class _TVertex>
			void CreateDeviceResources(ID3D11Device* pDevice, const _TVertex* vertices, unsigned int VerticesCount,IEffect *pEffect = nullptr, D3D_PRIMITIVE_TOPOLOGY primitiveTopology = D3D_PRIMITIVE_TOPOLOGY::D3D10_PRIMITIVE_TOPOLOGY_TRIANGLELIST, UINT VertexStride = sizeof(_TVertex), UINT startIndex = 0, UINT VertexOffset = 0);

			template<class _TVertex, class _TIndex>
			void CreateDeviceResources(ID3D11Device* pDevice, const _TVertex* vertices, unsigned int VerticesCount, const _TIndex* indices, unsigned int IndicesCount, IEffect *pEffect = nullptr, D3D_PRIMITIVE_TOPOLOGY primitiveTopology = D3D_PRIMITIVE_TOPOLOGY::D3D10_PRIMITIVE_TOPOLOGY_TRIANGLELIST, UINT VertexStride = sizeof(_TVertex), UINT startIndex = 0, UINT VertexOffset = 0);

			void CreateInputLayout(ID3D11Device* pDevice, IEffect *pEffect);
			void CreateInputLayout(ID3D11Device* pDevice, const void * pShaderByteCode, size_t pByteCodeLength);

			bool Empty() const { return VertexCount == 0; }
			template<class TVertex>
			void SetInputElementDescription()
			{
				static_assert(TVertex::InputElementCount, "Valiad Vertex Type should have static InputElements/InputElementCount member");
				InputElementCount = TVertex::InputElementCount;
				pInputElements = TVertex::InputElements;
				VertexStride = sizeof(TVertex);
			}

			typedef std::vector<std::unique_ptr<MeshBuffer>> Collection;

			uint32_t                                                IndexCount;
			uint32_t												VertexCount;
			uint32_t                                                StartIndex;
			uint32_t                                                VertexOffset;
			uint32_t                                                VertexStride;
			D3D_PRIMITIVE_TOPOLOGY                                  PrimitiveType;
			DXGI_FORMAT                                             IndexFormat;
			UINT													InputElementCount; // Vertex Description info
			const D3D11_INPUT_ELEMENT_DESC*							pInputElements; // Vertex Description info

			Microsoft::WRL::ComPtr<ID3D11InputLayout>               pInputLayout;
			Microsoft::WRL::ComPtr<ID3D11Buffer>                    pIndexBuffer;
			Microsoft::WRL::ComPtr<ID3D11Buffer>                    pVertexBuffer;

			// Setup the Vertex/Index Buffer and call the draw command
			void Draw(ID3D11DeviceContext *pContext, IEffect *pEffect = nullptr) const;
		};

		// Strong typed mesh buffer with static vertex typeing
		template<class _TVertex, class _TIndex>
		struct TypedMeshBuffer : public MeshBuffer
		{
		public:
			static_assert(std::is_integral<_TIndex>::value, "IndexType is not integer");
			static_assert(_TVertex::InputElementCount > 0, "Vertex type not fit the concept");

			typedef TypedMeshBuffer	SelfType;
			typedef _TVertex	VertexType;
			typedef _TIndex		IndexType;
		};

		namespace GeometricPrimtives
		{
			typedef TypedMeshBuffer<VertexPositionNormalTexture, uint16_t> MeshBufferType;
			std::shared_ptr<MeshBufferType>	CreateCube(ID3D11Device * pDevice, float size, bool rhcoords = true);
			std::shared_ptr<MeshBufferType>	CreateSphere(ID3D11Device * pDevice, float radius, size_t tessellation = 16, bool rhcoords = true, bool inside_facing = false);
			std::shared_ptr<MeshBufferType>	CreateCylinder(ID3D11Device * pDevice, float radius, float height, size_t tessellation = 32, bool rhcoords = true);
			std::shared_ptr<MeshBufferType>	CreateCone(ID3D11Device * pDevice, float radius, size_t tessellation = 32, bool rhcoords = true);
		}


		template<class _TVertex, class _TIndex>
		struct DynamicMeshBuffer : public TypedMeshBuffer<_TVertex, _TIndex>
		{
		public:
			typedef std::vector<VertexType> VertexCollectionType;
			typedef std::vector<IndexType>	IndexCollectionType;

			void UpdateMeshBuffer(ID3D11Device* pDevice);
			VertexCollectionType Vertices;
			IndexCollectionType	 Indices;
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
			std::shared_ptr<IMaterial>		pMaterial;	// Material of this model part
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
				LinearTransform	Transform;
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
			void AddChild(const std::shared_ptr<IModelNode> &model, const LinearTransform &transform = reinterpret_cast<const LinearTransform&>(LinearTransform::Identity));
			//void push_back(const value_type& _Val);
			//void push_back(value_type&& _Val);
			virtual void Render(ID3D11DeviceContext *pContext, const Matrix4x4& transform, IEffect* pEffect) override;

			const std::string& Name() const { return _Name; }
			void SetName(const std::string& name) { _Name = name; }

			// Inherited via IModelNode
			virtual BoundingBox GetBoundingBox() const override;
			//virtual BoundingOrientedBox GetOrientedBoundingBox() const override;

		};

		class IDynamicAsset
		{
		public:
			virtual ~IDynamicAsset();
			// Check if the GPU buffer is flaged as DYNAMIC
			virtual bool IsDeviceResourceDynamic() const;;

			// Check if the data have been loaded to CPU memery
			virtual bool IsInMemery() const;

			virtual bool IsInDevice() const;

			// Create GPU objects using Vertex/Index Buffer in CPU
			virtual bool CreateDeviceResource(ID3D11Device *pDevice) = 0;

			// Update GPU buffers using Vertex/Index Buffer in CPU
			virtual bool UpdateDeviceResource(ID3D11DeviceContext *pContext);

			// Release the Vertex/Index Buffer in CPU memery
			virtual void ReleaseDynamicResource();

			// To a amx binary stream , default to false
			virtual bool Serialize(std::ostream& binary) const;

			// from a amx binary stream
			virtual bool Deserialize(std::istream& binary);

			// Reload from file to CPU
			virtual bool Reload() = 0;
		};

		// This Model also keeps the geomreics data in CPU
		class DefaultStaticModel : public CompositionModel, public IDynamicAsset
		{
		public:
			typedef VertexPositionNormalTexture				VertexType;
			typedef uint16_t								IndexType;
			typedef FacetPrimitives::Triangle<IndexType>	TriangleType;

			static DefaultStaticModel * CreateFromObjFile(const std::wstring &file, ID3D11Device * pDevice = nullptr, const std::wstring& textureDir = L"");
			static DefaultStaticModel * CreateFromFbxFile(const std::wstring &file, ID3D11Device * pDevice = nullptr, const std::wstring& textureDir = L"");
			static DefaultStaticModel * CreateFromSmxFile(ID3D11Device *pDevice, const std::wstring &file);

			virtual void Render(ID3D11DeviceContext *pContext, const Matrix4x4& transform, IEffect* pEffect = nullptr) override;

			// Check if the data have been loaded to CPU memery
			bool IsInMemery() const override;
			// Create GPU objects using Vertex/Index Buffer in CPU
			bool CreateDeviceResource(ID3D11Device *pDevice) override;
			// Release the Vertex/Index Buffer in CPU memery
			void ReleaseDynamicResource() override;

			// To a obj text stream
			bool Serialize(std::ostream& binary) const override;
			// from a obj text stream
			bool Deserialize(std::istream& binary) override;

			// Reload from file to CPU
			bool Reload() override;

			DefaultStaticModel();
			~DefaultStaticModel();
		public:
			std::vector<VertexType>								Vertices;
			std::vector<FacetPrimitives::Triangle<IndexType>>	Facets;
			stdx::stride_range<Vector3>							Positions;
			stdx::stride_range<Vector3>							Normals;
			stdx::stride_range<Vector2>							TexCoords;

		private:
			bool				m_IsLoaded;
			const std::wstring  m_FilePath;
			const std::wstring  m_TexDir;
		};

		struct SkinMeshData;

		class ISkinningModel
		{
		public:
			virtual size_t GetBonesCount() const = 0;
			virtual DirectX::XMMATRIX* GetBoneTransforms() = 0;
			inline const DirectX::XMMATRIX* GetBoneTransforms() const
			{
				return const_cast<ISkinningModel*>(this)->GetBoneTransforms();
			}
		};

		class DefaultSkinningModel :
			public CompositionModel, public IDynamicAsset , public ISkinningModel
		{
		public:
			typedef DirectX::VertexPositionNormalTangentColorTextureSkinning VertexType;
			typedef uint16_t IndexType;
			typedef FacetPrimitives::Triangle<IndexType> TriangleType;

			static DefaultSkinningModel* CreateFromFbxFile(const std::string& file, const std::wstring& textureDir = L"", ID3D11Device* pDevice = nullptr);
			static DefaultSkinningModel* CreateFromAmxFile(const std::string& file, ID3D11Device* pDevice = nullptr);
			static DefaultSkinningModel* CreateFromData(SkinMeshData* pData, const std::wstring& textureDir = L"", ID3D11Device* pDevice = nullptr);
			static DefaultSkinningModel* CreateFromDatas(std::list<SkinMeshData>& datas, const std::wstring& textureDir = L"", ID3D11Device* pDevice = nullptr);

			// Check if the data have been loaded to CPU memery
			bool IsInMemery() const override;
			// Create GPU objects using Vertex/Index Buffer in CPU
			bool CreateDeviceResource(ID3D11Device *pDevice) override;
			// Release the Vertex/Index Buffer in CPU memery
			void ReleaseDynamicResource();

			// To a amx binary stream
			bool Serialize(std::ostream& binary) const override;
			// from a amx binary stream
			bool Deserialize(std::istream& binary) override;
			// Reload from file to CPU
			bool Reload() override;

			// IModelNode
			virtual void Render(ID3D11DeviceContext *pContext, const Matrix4x4& transform, IEffect* pEffect = nullptr) override;

			// Inherited via ISkinningModel
			virtual size_t GetBonesCount() const override;
			virtual DirectX::XMMATRIX* GetBoneTransforms() override;

			DefaultSkinningModel();
			~DefaultSkinningModel();
		public:
			std::vector<DirectX::Matrix4x4,
				DirectX::AlignedAllocator<DirectX::Matrix4x4, 16U >>
				BoneTransforms;

		public:
			stdx::stride_range<VertexType>							Vertices;
			stdx::stride_range<TriangleType>						Facets;
			stdx::stride_range<Vector3>								Positions;
			stdx::stride_range<Vector3>								Normals;
			stdx::stride_range<Vector2>								TexCoords;
			stdx::stride_range<Vector4>								Tagents;
			stdx::stride_range<uint32_t>							BlendWeights;
			stdx::stride_range<uint32_t>							BlendIndices;

		private:
			bool				m_IsLoaded;
			const std::string	m_FilePath;

			uint32_t	m_VertexCount;
			uint32_t	m_IndexCount;
			uint32_t	m_BonesCount;
			std::unique_ptr<VertexType[]>	m_Vertices;
			std::unique_ptr<IndexType[]>	m_Indices;

			void SetFromSkinMeshData(std::list<SkinMeshData> &meshes, const std::wstring& textureDir = L"");
			void SetFromSkinMeshData(SkinMeshData* pData, const std::wstring& textureDir = L"");
			void ResetRanges();

		};

		template <class _TVertex>
		inline void MeshBuffer::CreateDeviceResources(ID3D11Device * pDevice, const _TVertex * vertices, unsigned int VerticesCount, IEffect * pEffect, D3D_PRIMITIVE_TOPOLOGY primitiveTopology, UINT vertexStride, UINT startIndex, UINT vertexOffset)
		{
			CreateDeviceResources<_TVertex, int>(pDevice, vertices, VerticesCount, nullptr, 0, pEffect, primitiveTopology, vertexStride, startIndex, vertexOffset);
		}

		template <class _TVertex, class _TIndex>
		inline void MeshBuffer::CreateDeviceResources(ID3D11Device * pDevice, const _TVertex * vertices, unsigned int VerticesCount, const _TIndex * indices, unsigned int IndicesCount, IEffect * pEffect, D3D_PRIMITIVE_TOPOLOGY primitiveTopology, UINT vertexStride, UINT startIndex, UINT vertexOffset)
		{
			SetInputElementDescription<_TVertex>();

			assert(pDevice != nullptr);

			if (pEffect != nullptr)
			{
				CreateInputLayout(pDevice, pEffect);
			}


			pVertexBuffer = DirectX::CreateVertexBuffer<_TVertex>(pDevice, VerticesCount, vertices);
			if (indices != nullptr && IndicesCount > 0)
				pIndexBuffer = DirectX::CreateIndexBuffer(pDevice, IndicesCount, indices);
			IndexFormat = ExtractDXGIFormat<_TIndex>::value;
			VertexCount = VerticesCount;
			IndexCount = IndicesCount;
			StartIndex = startIndex;
			VertexStride = vertexStride;
			VertexOffset = vertexOffset;
			PrimitiveType = primitiveTopology;
		}
}
}

