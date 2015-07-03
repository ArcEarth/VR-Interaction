#include "pch_bcl.h"
#include "AssetDictionary.h"
#include <DirectXHelper.h>
#include "FbxParser.h"

using namespace Causality;
using namespace boost::filesystem;
using namespace DirectX;
using namespace DirectX::Scene;

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

AssetDictionary::mesh_type * AssetDictionary::LoadObjMesh(const string & key, const string & fileName)
{
	typedef DefaultStaticModel mesh_type;
	auto *pObjModel = DefaultStaticModel::CreateFromObjFile((mesh_directory / fileName).wstring(), render_device, texture_directory.wstring());
	if (!pObjModel)
		return nullptr;
	meshes[key] = pObjModel;
	auto& mesh = *pObjModel;
	for (auto& part : mesh.Parts)
	{
		//!+ This is A CHEAP HACK!!!! 
		part.pEffect = default_effect;
		if (!part.pMaterial)
			part.pMaterial = default_material;

		part.pMaterial->SetupEffect(part.pEffect.get());
		part.pMesh->pInputLayout = GetInputLayout<mesh_type::VertexType>(part.pEffect.get());
	}
	return meshes[key];
}

AssetDictionary::mesh_type * AssetDictionary::LoadFbxMesh(const string & key, const string & fileName, const std::shared_ptr<material_type> &pMaterial)
{
	typedef DefaultSkinningModel model_type;

	FbxParser fbx;
	fbx.ImportMesh((mesh_directory / fileName).string(),true); // with rewind
	auto datas = fbx.GetMeshs();
	if (datas.size() == 0)
		return nullptr;
	auto& mesh = datas.front();

	// Force clear Diffuse Map
	mesh.Material.DiffuseMapName = "";

	auto pModel = model_type::CreateFromData(&mesh, texture_directory.wstring(), render_device);
	//auto pModel = model_type::CreateCube(render_device,1.0f);

	pModel->SetName(fileName);
	pModel->pEffect = default_skinned_effect;

	if (pMaterial)
		pModel->pMaterial = pMaterial;

	if (!pModel->pMaterial)
		pModel->pMaterial = default_material;

	pModel->pMaterial->SetupEffect(pModel->pEffect.get());
	pModel->pMesh->pInputLayout = GetInputLayout<model_type::VertexType>(pModel->pEffect.get());

	meshes[key] = pModel;
	return meshes[key];
}

AssetDictionary::texture_type & AssetDictionary::LoadTexture(const string & key, const string & fileName)
{
	textures[key] = DirectX::Texture::CreateFromDDSFile(render_device, (texture_directory / fileName).c_str());
	return textures[key];
}

AssetDictionary::armature_type & AssetDictionary::LoadArmature(const string & key, const string & fileName)
{
	std::ifstream file((mesh_directory / fileName).wstring());
	auto pArmature = new armature_type(file);
	assets[key] = pArmature;
	return *pArmature;
}

AssetDictionary::animation_clip_type& AssetDictionary::LoadAnimation(const string & key, const string & fileName)
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

BehavierSpace * AssetDictionary::LoadBehavierFbx(const string & key, const string & fileName)
{
	FbxParser fbxparser;
	auto result = fbxparser.ImportBehavier((mesh_directory / fileName).string());
	BehavierSpace* behavier = nullptr;
	if (result)
	{
		behaviers[key] = fbxparser.GetBehavier();
		behavier = fbxparser.GetBehavier();
	}
	return behavier;
}

AssetDictionary::behavier_type * AssetDictionary::LoadBehavierFbxs(const string & key, const string & armature, list<std::pair<string, string>>& animations)
{
	FbxParser fbxparser;
	auto result = fbxparser.ImportBehavier((mesh_directory / armature).string());
	BehavierSpace* behavier = nullptr;
	if (result)
	{
		for (auto& anim : animations)
		{
			fbxparser.ImportAnimation((animation_directory / anim.second).string(), anim.first);
		}
		behaviers[key] = fbxparser.GetBehavier();
		behavier = fbxparser.GetBehavier();
	}
	return behavier;
}

task<AssetDictionary::mesh_type*>& AssetDictionary::LoadMeshAsync(const string & key, const string & fileName)
{
	using namespace std::placeholders;
	task<mesh_type*> load_mesh([this, key,fileName] () 
	{ 
		return this->LoadObjMesh(key,fileName);
	});
	loading_meshes[fileName] = std::move(load_mesh);
	return loading_meshes[fileName];
}

task<AssetDictionary::texture_type*>& AssetDictionary::LoadTextureAsync(const string & key, const string & fileName)
{
	task<texture_type*> loading_texture([this,key, fileName]() { return &this->LoadTexture(key,fileName); });
	loading_textures[key] = std::move(loading_texture);
	return loading_textures[key];
}

AssetDictionary::audio_clip_type & AssetDictionary::GetAudio(const string & key)
{
	return GetAsset<audio_clip_type>(key);
}

void AssetDictionary::SetRenderDevice(RenderDevice & device)
{
	render_device = device;
	if (!effect_factory)
	{
		effect_factory = std::make_unique<DirectX::EffectFactory>(device.Get());
		effect_factory->SetDirectory(texture_directory.c_str());

		default_effect = std::make_shared<DirectX::BasicEffect>(device.Get());
		default_effect->SetVertexColorEnabled(false);
		default_effect->SetTextureEnabled(true);
		default_effect->EnableDefaultLighting();

		//default_skinned_effect = default_effect;
		
		auto pSEffect = std::make_shared<DirectX::DGSLEffect>(device.Get(),nullptr,true);
		//auto pSEffect = std::make_shared<DirectX::SkinnedEffect>(device.Get());
		pSEffect->SetWeightsPerVertex(4U);
		pSEffect->SetVertexColorEnabled(true);
		pSEffect->SetTextureEnabled(false);
		pSEffect->EnableDefaultLighting();
		pSEffect->SetAlphaDiscardEnable(true);
		default_skinned_effect = pSEffect;

		default_material = std::make_unique<PhongMaterial>();
		default_material->pDefaultRequestEffect = default_effect.get();
		default_material->Name = "Default";

		effects["default"] = default_effect.get();
		effects["default_skinned"] = default_skinned_effect.get();
	}
}

void AssetDictionary::SetParentDictionary(AssetDictionary * dict)
{
	parent_dictionary = dict;
}

void AssetDictionary::SetTextureDirectory(const path & dir)
{
	texture_directory = dir;
}

void AssetDictionary::SetMeshDirectory(const path & dir)
{
	mesh_directory = dir;
}

void AssetDictionary::SetAssetDirectory(const path & dir)
{
	asset_directory = dir;
}
