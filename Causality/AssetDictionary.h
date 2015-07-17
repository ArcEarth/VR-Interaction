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
#include <new>
#include <unordered_map>

namespace Causality
{
	class AssetDictionary;

	using boost::filesystem::path;

	class asset_dictionary;

	class asset_base
	{
	friend asset_dictionary;

	protected:
		ptrdiff_t					_ref_count;
		asset_dictionary*			_dictionary;
		path						_ref_path;
		bool						_loaded;
		virtual						~asset_base(){}
	public:

		asset_dictionary*			dictionary() { return _dictionary; }
		const asset_dictionary*		dictionary() const { return _dictionary; }

		const path&					source_path() const { return _ref_path; }
		bool						is_loaded() { return _loaded; }

		ptrdiff_t					ref_count() const { return _ref_count; }
	};

	template <class T>
	class asset;

	template <class T>
	class asset : public asset_base
	{
	private:
		union
		{
			T	_Tdata;
			char _Bytes[sizeof(T)];
		};

		friend asset_dictionary;

	public:
		virtual ~asset()
		{
			_Tdata.~T();
		}

		T&						get() { return _Tdata; }
		asset_ptr<T>			get_ptr() { return &_Tdata; }

		template<class... _Types> inline
		T*						internal_construct(_Types&&... _Args)
		{	// make a unique_ptr
			return new (&_Tdata) T(_STD forward<_Types>(_Args)...);
		}

		ptrdiff_t				inc_ref() { return ++_ref_count; }
		ptrdiff_t				dec_ref() 
		{ 
			return --_ref_count;
		}
	};

	template <class T>
	class asset_ptr
	{
	private:
		asset<T>* _ptr;
	public:
		T* get() { return  _ptr->get_ptr(); }
		const T* get() const { return  _ptr->get_ptr(); }
		operator T*() { return _ptr->get_ptr(); }
		operator const T*() const { return _ptr->get_ptr(); }

		asset_ptr() : _ptr(nullptr) {}
		~asset_ptr()
		{
			reset();
		}

		asset_ptr& operator=(const asset_ptr& rhs) 
		{
			_ptr = rhs._ptr; if (_ptr) _ptr->inc_ref(); return *this;
		}

		asset_ptr& operator=(asset_ptr&& rhs)
		{
			swap(rhs);
			return *this;
		}

		asset_ptr(const asset_ptr& rhs) { *this = rhs; }
		asset_ptr(asset_ptr&& rhs) { *this = std::move(rhs); }

		void reset() {
			if (_ptr && !_ptr->dec_ref())
				delete _ptr;
		}

		void attach(asset<T>* ptr)
		{
			reset();
			_ptr = ptr;
		}

		void swap(asset_ptr& rhs)
		{
			auto ptr = _ptr;
			_ptr = rhs._ptr;
			rhs._ptr = ptr;
		}
	public:
		asset_dictionary*		dictionary() { return _ptr->dictionary(); }
		const asset_dictionary*	dictionary() const { return _ptr->dictionary(); }

		const path&				source_path() const { return _ptr->source_path(); }
		bool					is_loaded() { return _ptr->is_loaded(); }

		ptrdiff_t				ref_count() const { return _ptr->ref_count(); }
	};

	class asset_dictionary
	{
	public:
		using mesh_type = DirectX::Scene::IModelNode;
		using texture_type = DirectX::Texture;
		using audio_clip_type = int;
		using animation_clip_type = ArmatureFrameAnimation;
		using behavier_type = BehavierSpace;
		using armature_type = StaticArmature;
		using effect_type = DirectX::IEffect;
		using material_type = DirectX::Scene::IMaterial;

		typedef std::function<bool(void*, const char*)> creation_function_type;
		map<std::type_index, creation_function_type> creators;

	public:
		// Helper for lazily creating a D3D resource.
		template<typename T, typename TCreateFunc>
		static T* demand_create(asset_ptr<T>& assetPtr, std::mutex& mutex, TCreateFunc createFunc)
		{
			T* result = assetPtr.get();

			// Double-checked lock pattern.
			MemoryBarrier();

			if (!result)
			{
				std::lock_guard<std::mutex> lock(mutex);

				result = comPtr.Get();

				if (!result)
				{
					// Create the new object.
					ThrowIfFailed(
						createFunc(&assetPtr)
						);

					MemoryBarrier();

					assetPtr.attach(result);
				}
			}

			return result;
		}


		template<class _Ty, class... _Types> inline
		asset_ptr<_Ty> make_asset(const string& key, _Types&&... _Args)
		{	
			auto ptr = new asset<_Ty>(std::forward(_Args)...);
		}

		template<class _Ty>
		asset_ptr<_Ty> get_asset(const string& key);

		template<>
		asset_ptr<mesh_type> get_asset(const string& key);

	private:
		std::unordered_map<string, asset_base*> _resources;
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

		const sptr<material_type>&	GetMaterial(const string& key) const
		{
			return materials[key];
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

		material_type*				AddMaterial(const string& key, const sptr<material_type>& pMaterial)
		{
			materials[key] = pMaterial;
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
		mutable map<string, sptr<material_type>>	materials;

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