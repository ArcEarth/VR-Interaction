#pragma once
#include "BCL.h"
#include "Textures.h"
#include "Models.h"
#include <boost\filesystem.hpp>
#include "Armature.h"
#include "CharacterBehavier.h"
#include "RenderContext.h"
#include <Effects.h>
#include <ppltasks.h>
#include <typeinfo>
#include "FbxParser.h"

namespace Causality
{
	class AssetDictionary;

	class asset_base
	{
	public:
		AssetDictionary*		dictionary() { return _dictionary; }
		string					source();
		bool					is_loaded() { return _loaded; }

	protected:
		size_t						_ref_count;
		AssetDictionary*			_dictionary;
		bool						_loaded;
		string						_ref_path;
	};

	template <class T>
	class asset;

	template <class T>
	class asset_ptr
	{
	public:
		T* get() { return  &_ptr->get(); }
		const T* get() const { return  &_ptr->get(); }
		operator T*() { return _ptr; }
		operator const T*() const { return _ptr; }

	private:
		asset<T>* _ptr;
	};

	template <class T>
	class asset : public asset_base
	{
	public:
		T&						get() { return _Tdata; }
		asset_ptr<T>			get_ptr();

	private:
		union
		{
			T	_Tdata;
			char _Bytes[sizeof(T)];
		};
	};

	class asset_dictionary
	{
		typedef std::function<bool(void*, const char*)> creation_function_type;
		map<std::type_index, creation_function_type> creators;
	private:
		map<string, void*> _resources;
	};

	using concurrency::task;

	// Controls the asset name resolve and loading
	class AssetDictionary
	{
	public:
		using path = boost::filesystem::path;
		using mesh_type = DirectX::Scene::IModelNode;
		using texture_type = DirectX::Texture;
		using audio_clip_type = int;
		using animation_clip_type = ArmatureFrameAnimation;
		using behavier_type = BehavierSpace;
		using armature_type = StaticArmature;
		using effect_type = DirectX::IEffect;
		using material_type = DirectX::Scene::IMaterial;

		AssetDictionary();
		~AssetDictionary();

		//Synchronize loading methods
		mesh_type*		     LoadObjMesh(const string & key, const string& fileName);
		mesh_type*		     LoadFbxMesh(const string & key, const string& fileName, const std::shared_ptr<material_type> &pMaterial = nullptr);
		texture_type&	     LoadTexture(const string & key, const string& fileName);
		armature_type&	     LoadArmature(const string & key, const string& fileName);
		animation_clip_type& LoadAnimation(const string& key, const string& fileName);
		behavier_type*		 LoadBehavierFbx(const string & key, const string & fileName);
		behavier_type*		 LoadBehavierFbxs(const string & key, const string& armature, list<std::pair<string,string>>& animations);

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

		mesh_type*					GetMesh(const string& key)
		{
			auto itr = meshes.find(key);
			if (itr != meshes.end())
				return itr->second;
			return nullptr;
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
		const cptr<ID3D11InputLayout>& GetInputLayout(DirectX::IEffect* pEffct = nullptr);

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
		const path& GetTextureDirectory() const { return texture_directory; }
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

		map<string, mesh_type*>				meshes;
		map<string, texture_type>			textures;
		map<string, animation_clip_type>	animations;
		map<string, audio_clip_type>		audios;
		map<string, effect_type*>			effects;
		map<string, behavier_type*>			behaviers;

		sptr<DirectX::IEffect>				default_effect;
		sptr<DirectX::IEffect>				default_skinned_effect;
		sptr<DirectX::Scene::PhongMaterial>	default_material;
		uptr<DirectX::EffectFactory>		effect_factory;

		map<std::type_index, cptr<ID3D11InputLayout>> layouts;

		// other assets
		map<string, any>					assets;

	public:

		auto GetEffects()
		{
			return adaptors::values(effects);
		}
	};

	template <typename VertexType>
	inline const cptr<ID3D11InputLayout>& AssetDictionary::GetInputLayout(DirectX::IEffect * pEffct)
	{
		std::type_index type = typeid(VertexType);
		auto& pLayout = layouts[type];
		if (pLayout == nullptr)
		{
			void const* shaderByteCode;
			size_t byteCodeLength;
			pEffct->GetVertexShaderBytecode(&shaderByteCode, &byteCodeLength);
			pLayout = DirectX::CreateInputLayout<VertexType>(render_device.Get(), shaderByteCode, byteCodeLength);
		}
		return pLayout;
	}

}