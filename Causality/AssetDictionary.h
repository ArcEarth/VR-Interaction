#pragma once
#include "BCL.h"
#include "Common\Textures.h"
#include "Common\Model.h"
#include <boost\filesystem.hpp>
#include "Armature.h"
#include "RenderContext.h"
#include <Effects.h>
namespace Causality
{
	class AssetDictionary;

	struct AssetReference
	{
	public:
		AssetDictionary&		Dictionary;
		boost::filesystem::path Source;

		bool IsLoaded() const;
	};

	// Controls the asset name resolve and loading
	class AssetDictionary
	{
	public:
		using path = boost::filesystem::path;
		using mesh_type = DirectX::Scene::GeometryModel;
		using texture_type = DirectX::Texture;
		using audio_clip_type = int;
		using animation_clip_type = ArmatureFrameAnimation;
		using behavier_type = BehavierSpace;
		using armature_type = StaticArmature;
		using effect_type = DirectX::IEffect;

		AssetDictionary();
		~AssetDictionary();

		//Synchronize loading methods
		mesh_type&		     LoadMesh(const string & key, const string& fileName);
		texture_type&	     LoadTexture(const string & key, const string& fileName);
		armature_type&	     LoadArmature(const string & key, const string& fileName);
		animation_clip_type& LoadAnimation(const string& key, const string& fileName);
		behavier_type&		 LoadBehavierFbx(const string & key, const string & fileName);

		// Async loading methods
		task<mesh_type*>&			LoadMeshAsync(const string & key, const string& fileName);
		task<texture_type*>&		LoadTextureAsync(const string & key, const string& fileName);
		task<audio_clip_type*>&		LoadAudioAsync(const string & key, const string& fileName);

		bool IsAssetLoaded(const string& key) const;

		template<typename TAsset>
		TAsset&						GetAsset(const string& key)
		{
			return *any_cast<TAsset*>(assets[key]);
		}

		mesh_type&					GetMesh(const string& key)
		{
			auto itr = meshes.find(key);
			if (itr != meshes.end())
				return itr->second;
			return itr->second;
		}

		texture_type&				GetTexture(const string& key)
		{
			auto itr = textures.find(key);
			if (itr != textures.end())
				return itr->second;
			return itr->second;
		}

		animation_clip_type&		GetAnimation(const string& key)
		{
			return animations[key];
		}

		behavier_type&				GetBehavier(const string& key)
		{
			return *behaviers[key];
		}

		audio_clip_type&			GetAudio(const string& key);

		effect_type*				GetEffect(const string& key)
		{
			return default_effect.get();
		}

		template<typename VertexType>
		const cptr<ID3D11InputLayout>& GetInputLayout()
		{
			return pInputLayout;
		}

		DirectX::EffectFactory&		EffctFactory() { return *effect_factory; }

		template<typename TAsset>
		TAsset&						AddAsset(const string&key, TAsset* pAsset)
		{
			assets[key] = pAsset;
			return *pAsset;
		}

		template<typename TAsset>
		TAsset&						LoadAsset(const string& fileName, function<TAsset(const string& fileName, std::istream& file)> deserializer)
		{
		}

		template<typename TAsset>
		task<TAsset&>&				LoadAssetAsync(const string& fileName, function<TAsset(const string& fileName, std::istream& file)> deserializer)
		{
		}

		RenderDevice& GetRenderDevice() { return render_device; }
		const RenderDevice& GetRenderDevice() const { return render_device; }
		void SetRenderDevice(RenderDevice& device);
		void SetParentDictionary(AssetDictionary* dict);
		void SetTextureDirectory(const path& dir);
		void SetMeshDirectory(const path& dir);
		void SetAssetDirectory(const path& dir);

	private:
		RenderDevice						render_device;

		AssetDictionary*					parent_dictionary;

		path								asset_directory;
		path								texture_directory;
		path								mesh_directory;
		path								animation_directory;

		map<string, task<mesh_type*>>		loading_meshes;
		map<string, task<texture_type*>>	loading_textures;

		map<string, mesh_type>				meshes;
		map<string, texture_type>			textures;
		map<string, animation_clip_type>	animations;
		map<string, audio_clip_type>		audios;
		map<string, effect_type*>			effects;
		map<string, behavier_type*>			behaviers;

		sptr<DirectX::BasicEffect>			default_effect;
		uptr<DirectX::EffectFactory>		effect_factory;
		cptr<ID3D11InputLayout>				pInputLayout;
		// other assets
		map<string, any>					assets;
	};

	struct MeshReference : AssetReference
	{
		AssetDictionary::mesh_type& Get();
	};

	struct TextureReference : AssetReference
	{
		AssetDictionary::texture_type& Get();
	};

	struct AudioReference : AssetReference
	{
		AssetDictionary::audio_clip_type& Get();
	};

}