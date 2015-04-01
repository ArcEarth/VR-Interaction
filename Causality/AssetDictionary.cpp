#include "pch_bcl.h"
#include "AssetDictionary.h"
#include "Common\DirectXHelper.h"

using namespace Causality;
using namespace boost::filesystem;
AssetDictionary::AssetDictionary()
{
	asset_directory = current_path() / "Resources";
	mesh_directory = asset_directory / "Models";
	texture_directory = asset_directory / "Textures";
	animation_directory = asset_directory / "Animations";
}


AssetDictionary::~AssetDictionary()
{
}

inline std::wstring towstring(const string& str)
{
	return std::wstring(str.begin(), str.end());
}

AssetDictionary::mesh_type & Causality::AssetDictionary::LoadMesh(const string & key, const string & fileName)
{
	mesh_type::CreateFromObjFile(&meshes[key], render_device, (mesh_directory / fileName).wstring(), texture_directory.wstring());
	auto& mesh = meshes[key];
	for (auto& part : mesh.Parts)
	{
		//!+ This is A CHEAP HACK!!!! 
		part.pEffect = default_effect;
		if (part.pMaterial)
			part.pMaterial->Effect = GetEffect(part.pMaterial->Name);
		part.pMesh->pInputLayout = GetInputLayout<AssetDictionary::mesh_type::VertexType>();
	}
	return meshes[key];
}

AssetDictionary::texture_type & Causality::AssetDictionary::LoadTexture(const string & key, const string & fileName)
{
	textures[key] = DirectX::Texture::CreateFromDDSFile(render_device, (texture_directory / fileName).c_str());
	return textures[key];
}

AssetDictionary::armature_type & Causality::AssetDictionary::LoadArmature(const string & key, const string & fileName)
{
	std::ifstream file((mesh_directory / fileName).wstring());
	auto pArmature = new armature_type(file);
	assets[key] = pArmature;
	return *pArmature;
}

AssetDictionary::animation_clip_type& Causality::AssetDictionary::LoadAnimation(const string & key, const string & fileName)
{
	using clip_type = AssetDictionary::animation_clip_type;
	std::ifstream stream((animation_directory / fileName).wstring());
	if (stream.is_open())
	{
		animations.emplace(key, clip_type(stream));
		return animations[key];
	}
	return animations[key];
}

task<AssetDictionary::mesh_type*>& Causality::AssetDictionary::LoadMeshAsync(const string & key, const string & fileName)
{
	using namespace std::placeholders;
	task<mesh_type*> load_mesh([this, key,fileName] () { return &this->LoadMesh(key,fileName); });
	loading_meshes[fileName] = std::move(load_mesh);
	return loading_meshes[fileName];
}

task<AssetDictionary::texture_type*>& Causality::AssetDictionary::LoadTextureAsync(const string & key, const string & fileName)
{
	task<texture_type*> loading_texture([this,key, fileName]() { return &this->LoadTexture(key,fileName); });
	loading_textures[key] = std::move(loading_texture);
	return loading_textures[key];
}

AssetDictionary::audio_clip_type & Causality::AssetDictionary::GetAudio(const string & key)
{
	return GetAsset<audio_clip_type>(key);
}

void Causality::AssetDictionary::SetRenderDevice(RenderDevice & device)
{
	render_device = device;
	if (!effect_factory)
	{
		effect_factory = std::make_unique<DirectX::EffectFactory>(device.Get());
		default_effect = std::make_shared<DirectX::BasicEffect>(device.Get());
		default_effect->SetVertexColorEnabled(false);
		default_effect->SetTextureEnabled(true);
		//default_effect->SetLightingEnabled(true);
		default_effect->EnableDefaultLighting();
		{
			void const* shaderByteCode;
			size_t byteCodeLength;
			default_effect->GetVertexShaderBytecode(&shaderByteCode, &byteCodeLength);
			pInputLayout = DirectX::CreateInputLayout<DirectX::VertexPositionNormalTexture>(device, shaderByteCode, byteCodeLength);
		}
	}
}

void Causality::AssetDictionary::SetParentDictionary(AssetDictionary * dict)
{
	parent_dictionary = dict;
}

void Causality::AssetDictionary::SetTextureDirectory(const path & dir)
{
	texture_directory = dir;
}

void Causality::AssetDictionary::SetMeshDirectory(const path & dir)
{
	mesh_directory = dir;
}

void Causality::AssetDictionary::SetAssetDirectory(const path & dir)
{
	asset_directory = dir;
}