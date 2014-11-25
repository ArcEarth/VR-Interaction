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

		class IMaterial abstract
		{
		public:
			virtual ~IMaterial() {}

			virtual Color	GetColor(const char* key) const = 0;
			virtual float	GetFloat(const char* key) const = 0;
			virtual int		GetInt(const char* key) const = 0;
			virtual int		GetString(const char* key) const = 0;
			virtual ID3D11ShaderResourceView*	GetTexture(const char* key) const = 0;
			//virtual Color	SetColor(const char* key) const = 0;
			//virtual void SetFloat(const char* key) const = 0;
			//virtual void SetInt(const char* key) const = 0;
			//virtual void SetString(const char* key) const = 0;
			//virtual void SetTexture(const char* key) const = 0;
		};

		class IPhongMaterial abstract
		{
			// Phong model
			virtual Color GetAmbientColor() const = 0;
			virtual Color SetAmbientColor() = 0;
			virtual Color GetDiffuseColor() const = 0;
			virtual Color SetDiffuseColor() = 0;
			virtual Color GetSpecularColor() const = 0;
			virtual Color SetSpecularColor() = 0;
			virtual float GetAlpha() const = 0;
			virtual float SetAlpha() const = 0;

			virtual ID3D11ShaderResourceView *GetDiffuseMap() const = 0;
			virtual ID3D11ShaderResourceView *GetBumpMap() const = 0;
			virtual ID3D11ShaderResourceView *GetDisplaceMap() const = 0;
		};

		struct ObjMaterial : public IPhongMaterial , public IMaterial
		{
			ObjMaterial();
			static std::vector<ObjMaterial> LoadFromFile(ID3D11Device* pDevice, const std::wstring &file, const std::wstring &lookupDirectory);

			const std::string Name;
			Color AmbientColor;
			Color DiffuseColor;
			Color SpecularColor;
			float Alpha;

			ComPtr<ID3D11ShaderResourceView> DiffuseMap;
			ComPtr<ID3D11ShaderResourceView> BumpMap;
			ComPtr<ID3D11ShaderResourceView> DisplaceMap;

			// Inherited via IMaterial
			virtual Color GetAmbientColor() const override;
			virtual Color GetDiffuseColor() const override;
			virtual Color GetSpecularColor() const override;
			virtual float GetAlpha() const override;
			virtual ID3D11ShaderResourceView * GetDiffuseMap() const override;
			virtual ID3D11ShaderResourceView * GetBumpMap() const override;
			virtual ID3D11ShaderResourceView * GetDisplaceMap() const override;
		};

		// A Container of Vertex and Indices holding geometry information with Identical effect to render
		// Should be use with std::shared_ptr
		struct Mesh : public IMesh
		{
		public:
			~Mesh(){}

			template<class _TVertex, class _TIndex>
			void Update(ID3D11Device* pDevice, const _TVertex* vertices, unsigned int VerticesCount, const _TIndex* indices, unsigned int IndicesCount, D3D_PRIMITIVE_TOPOLOGY primitiveTopology = D3D_PRIMITIVE_TOPOLOGY::D3D10_PRIMITIVE_TOPOLOGY_TRIANGLELIST, size_t VertexStride = sizeof(_TVertex), UINT startIndex = 0, UINT VertexOffset = 0 , const std::shared_ptr<IEffect> &pRenderEffect = nullptr)
			{
				static_assert(std::is_integral<_TIndex>::value, "Type of Index must be integral type");
				static_assert(_TVertex::InputElementCount, "Valiad Vertex Type should have static InputElements/InputElementCount member");
				assert(pDevice != nullptr);

				if (pRenderEffect != nullptr)
				{
					pEffect = pRenderEffect;
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
			Microsoft::WRL::ComPtr<ID3D11InputLayout>               pInputLayout;
			Microsoft::WRL::ComPtr<ID3D11Buffer>                    pIndexBuffer;
			Microsoft::WRL::ComPtr<ID3D11Buffer>                    pVertexBuffer;
			std::shared_ptr<IEffect>                                pEffect;
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
			static ComPtr<ID3D11InputLayout> CreateInputLayout(ID3D11Device* pDevice,IEffect* pEffect)
			{
				void const* shaderByteCode;
				size_t byteCodeLength;
				pEffect->GetVertexShaderBytecode(&shaderByteCode, &byteCodeLength);
				pInputLayout = DirectX::CreateInputLayout(pDevice, shaderByteCode, byteCodeLength);
			}

			// Setup the effect for rendering and change the input layout as well
			void SetEffect(const std::shared_ptr<IEffect> &pRenderEffect)
			{
				pEffect = pRenderEffect;
			}

			typedef TypedMesh	SelfType;
			typedef _TVertex	VertexType;
			typedef _TIndex		IndexType;
		};

		class IModelPart
		{
			virtual void Draw(ID3D11DeviceContext *pContext, IEffect* pEffect) = 0;
		};
		// A ModelPart is a aggregate of a mesh and it's Material
		class ModelPart : IModelPart
		{ 
			Mesh mesh;
			
		};

		
		class Model : public RigidObject
		{
		public:

			std::vector<std::unique_ptr<Mesh>>	Meshs;
			std::vector<std::unique_ptr<IMaterial>> Materias;
		};

		// A Mesh that also keeps the vertices and facets information in CPU
		template<class _TVertex = VertexPositionNormalTexture, class _TIndex = uint16_t>
		struct GeometryMesh : public ModelMesh<_TVertex, _TIndex>
		{
		public:
			~GeometryMesh() {}
			static_assert(VertexTraits::has_position<VertexType>::value, "VertexType::position dont't exist.");

			typedef GeometryMesh SelfType;

			void Update(ID3D11Device *pDevice)
			{
				Mesh::Update(pDevice, &Vertices[0], Vertices.size(), &Facets[0][0], Facets.size()*3U, D3D_PRIMITIVE_TOPOLOGY::D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
			}

			std::vector<VertexType>	Vertices;
			std::vector<FacetPrimitives::Triangle<IndexType>>	Facets;
			//stride_range<IndexType> Indices;
		};

		class ObjMesh : public GeometryMesh<VertexPositionNormalTexture, uint16_t>
		{
		public:
			~ObjMesh()
			{}
			ObjMesh();
			ObjMesh(const std::wstring &file);

		protected:
			void GenerateNormal();

		public:
			stride_range<Vector3>	Positions;
			stride_range<Vector3>	Normals;
			stride_range<Vector2>	TexCoords;
		};

		//0. Color on and Ambient off
		//1. Color on and Ambient on
		//2. Highlight on
		//3. Reflection on and Ray trace on
		//4. Transparency: Glass on, Reflection : Ray trace on
		//5. Reflection : Fresnel on and Ray trace on
		//6. Transparency : Refraction on, Reflection : Fresnel off and Ray trace on
		//7. Transparency : Refraction on, Reflection : Fresnel on and Ray trace on
		//8. Reflection on and Ray trace off
		//9. Transparency : Glass on, Reflection : Ray trace off
		//10. Casts shadows onto invisible surfaces
		enum ObjMateriaIlluminitionModel
		{
			ColorOnAmbientOff = 0,
			ColorOnAmbientOn = 1,
			HighlightOn = 2,
			ReflectionOnRayTraceOn = 3,
			TransparencyOn = 4,
		};

		// Represent a model that can be describe by a obj file.
		// IMesh, IMaterial, IRigid
		class ObjModel : public ObjMesh , public ObjMaterial
		{
		public:
			~ObjModel()
			{
			}

			ObjModel(ID3D11Device* pDevice, const std::wstring &file, const std::wstring &textureDirectory, const std::shared_ptr<IEffect>& pEffect = nullptr);
		};
	}
}

