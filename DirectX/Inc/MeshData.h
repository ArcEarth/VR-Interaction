#pragma once
#include <VertexTypes.h>
#include <DirectXMath.h>
#include <string>
#include <ios>
#include "MaterialData.h"

namespace DirectX
{
	namespace Scene
	{
		struct StaticMeshData
		{
			typedef DirectX::VertexPositionNormalTangentColorTexture VertexType;
			typedef uint16_t IndexType;
			static const size_t PolygonSize = 3U;

			uint32_t	VertexCount;
			uint32_t	IndexCount;
			VertexType* Vertices;
			IndexType*	Indices;
			std::string Name;
		};

		/// <summary>
		///	Binary layout
		/// </summary>
		struct SkinMeshData
		{
			typedef DirectX::VertexPositionNormalTangentColorTextureSkinning VertexType;
			typedef uint16_t IndexType;
			static const size_t PolygonSize = 3U;

			std::string			Name;
			uint32_t			VertexCount;
			uint32_t			IndexCount;
			uint32_t			BonesCount;
			VertexType*			Vertices;
			IndexType*			Indices;
			PhongMaterialData	Material;

			SkinMeshData();
			// Release the internal Vertex/Index memery
			void Release();

			void Serialize(std::ostream& binary) const;
			void Deserialize(std::istream& binary);
		};
	}
}