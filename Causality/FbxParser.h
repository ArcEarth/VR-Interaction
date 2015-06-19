#pragma once
#include "BCL.h"
#include "Armature.h"
#include "CharacterBehavier.h"
#include <VertexTypes.h>

namespace Causality
{
	/// <summary>
	///	Binary layout
	/// </summary>
	struct SkinMeshData
	{
		typedef DirectX::VertexPositionNormalTangentColorTextureSkinning VertexType;
		typedef uint16_t IndexType;
		static const size_t PolygonSize = 3U;

		uint32_t	VertexCount;
		uint32_t	IndexCount;
		uint32_t	BonesCount;
		VertexType* Vertices;
		IndexType*	Indices;
		std::string Name;

		SkinMeshData *ParentMesh;
		DirectX::AffineTransform TransformToParent;

		SkinMeshData();
		// Release the internal Vertex/Index memery
		void Release();

		void Serialize(std::ostream& binary) const;
		void Deserialize(std::istream& binary);
	};

	class BinaryFileLoader
	{
		ArmatureFrameAnimation LoadAnimationClipFromFile(const string& filename);
		SkinMeshData			   LoadSkinnedMeshFromFile(const string& filename);
	};

	class FbxAnimationParser
	{
	public:
		enum class Mode : unsigned
		{
			None = 0U,
			ImportMeshs = 1U,
			ImportAnimations = 2U,
			CreateBehavierAndArmature = 4U,
			ImportArmature = 8U
		};

		explicit FbxAnimationParser(const string& file, unsigned mode);

		void SetBehavierProfile(BehavierSpace* pBehav);

		bool Load(const string& file, unsigned mode);
		bool ImportBehavier(const string& file);
		bool ImportMesh(const string& file);
		bool ImportAnimation(const string& file,const string& name);

		const std::list<SkinMeshData>&	GetMeshs();
		StaticArmature*					GetArmature();
		BehavierSpace*					GetBehavier();

		~FbxAnimationParser();
		FbxAnimationParser();
	private:
		struct Impl;
		unique_ptr<Impl> m_pImpl;
	};
}