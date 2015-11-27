#include "pch_bcl.h"
#include "AssetDictionary.h"
#include <DirectXHelper.h>
#include "FbxParser.h"
#include <ShadowMapEffect.h>
#include <boost\filesystem.hpp>

#include "Textures.h"
#include "Models.h"
#include "Armature.h"
#include "CharacterBehavier.h"
#include <Effects.h>


using namespace Causality;
using namespace boost::filesystem;
using namespace DirectX;
using namespace DirectX::Scene;
using boost::any;

AssetDictionary::AssetDictionary()
{
	asset_directory = current_path() / "Resources";
	mesh_directory = asset_directory / "Models";
	texture_directory = asset_directory / "Textures";
	animation_directory = asset_directory / "Animations";
}

template <typename T>
inline static void internal_delete(T* &pData) {
	if (pData) {
		delete pData;
		pData = nullptr;
	}
}

AssetDictionary::~AssetDictionary()
{
	for (auto& pair : behaviers)
		internal_delete(pair.second);
	for (auto& pair : meshes)
		internal_delete(pair.second);
	//for (auto& pair : materials)
	//	internal_delete(pair.second);
	for (auto& pair : textures)
		internal_delete(pair.second);
	//for (auto& pair : effects)
	//	internal_delete(pair.second);
}

void AssetDictionary::ParseArchive(const ParamArchive * store)
{
	store = GetFirstChildArchive(store);
	while (store)
	{
		const char* type = GetArchiveName(store);
		if (!strcmp(type, "mesh"))
			ParseMesh(store);
		else if (!strcmp(type, "texture"))
			ParseTexture(store);
		else if (!strcmp(type, "audio_clip"))
			ParseAudio(store);
		else if (!strcmp(type, "armature"))
			ParseArmature(store);
		else if (!strcmp(type, "behavier"))
			ParseBehavier(store);
		else if (!strcmp(type, "animation_clip"))
			ParseAnimation(store);
		else if (!strcmp(type, "phong_material"))
			ParseMaterial(store);

		store = GetNextSiblingArchive(store);
	}
}

AssetDictionary::audio_clip_type * AssetDictionary::ParseAudio(const ParamArchive * store)
{
	return nullptr;
}

AssetDictionary::mesh_type * AssetDictionary::ParseMesh(const ParamArchive * store)
{
	auto type = GetArchiveName(store);
	const char* name = nullptr;
	const char* src = nullptr;
	const char* mat = nullptr;
	GetParam(store, "src", src);
	GetParam(store, "name", name);
	GetParam(store, "material", mat);

	if (!strcmp(type, "box"))
	{
	}
	else if (!strcmp(type, "cylinder"))
	{
	}
	else if (!strcmp(type, "sphere"))
	{
	}
	else if (!strcmp(type, "mesh"))
	{
		if (src != nullptr && strlen(src) != 0)
		{
			boost::filesystem::path ref(src);
			if (ref.extension().string() == ".obj")
			{
				auto mesh = LoadObjMesh(name, src);
				return mesh;
			}
			else if (ref.extension().string() == ".fbx")
			{
				sptr<IMaterial> pMat;

				// Material overhaul
				if (mat != nullptr && mat[0] == '{') // asset reference
				{
					const std::string key(mat + 1, mat + strlen(mat) - 1);
					pMat = GetMaterial(key);
				}

				if (pMat != nullptr)
				{
					auto mesh = LoadFbxMesh(name, src, pMat);
					return mesh;
				}
				else
				{
					// import material
					auto mesh = LoadFbxMesh(name, src, true);
					return mesh;
				}
			}
		}
	}
	return GetMesh("default");
}

sptr<AssetDictionary::material_type> AssetDictionary::ParseMaterial(const ParamArchive * store)
{
	assert(!strcmp(GetArchiveName(store), "phong_material"));

	DirectX::Scene::PhongMaterialData data;
	GetParam(store,"name", data.Name);

	GetParam(store, "diffuse_map", data.DiffuseMapName);
	GetParam(store, "normal_map", data.NormalMapName);
	GetParam(store, "ambient_map", data.AmbientMapName);
	GetParam(store, "displace_map", data.DisplaceMapName);
	GetParam(store, "specular_map", data.SpecularMapName);
	GetParam(store, "alpha_discard", data.UseAlphaDiscard);

	GetParam(store, "diffuse_color", data.DiffuseColor);
	GetParam(store, "ambient_color", data.AmbientColor);
	GetParam(store, "specular_color", data.SpecularColor);
	GetParam(store, "emissive_color", data.EmissiveColor);
	GetParam(store, "reflection_color", data.RelfectionColor);

	auto pPhong = std::make_shared<DirectX::Scene::PhongMaterial>(data, GetTextureDirectory().wstring(), GetRenderDevice());
	AddMaterial(data.Name, pPhong);
	return pPhong;
}

AssetDictionary::texture_type * AssetDictionary::ParseTexture(const ParamArchive * store)
{
	const char* src = nullptr, * name = nullptr;
	GetParam(store, "name", name);
	GetParam(store, "src", src);

	if (!strcmp(GetArchiveName(store), "texture"))
	{
		return LoadTexture(name, src);
	}
	return GetTexture("default");
}

AssetDictionary::armature_type * AssetDictionary::ParseArmature(const ParamArchive * store)
{
	using armature_type = AssetDictionary::armature_type;
	const char* src = nullptr, *name = nullptr;
	GetParam(store, "name", name);
	GetParam(store, "src", src);
	if (src != nullptr)
	{
		return LoadArmature(name, src);
	}
	else // no file armature define
	{
		GetFirstChildArchive(store,"joint");
	}
	return nullptr;//GetAsset<armature_type>("default");
}

AssetDictionary::animation_clip_type * AssetDictionary::ParseAnimation(const ParamArchive * store)
{
	using clip_type = AssetDictionary::animation_clip_type;
	const char* src = nullptr, *name = nullptr;
	GetParam(store, "name", name);
	GetParam(store, "src", src);
	return LoadAnimation(name, src);
}


AssetDictionary::behavier_type * AssetDictionary::ParseBehavier(const ParamArchive * store)
{
	using behavier_type = AssetDictionary::behavier_type;
	const char* src = nullptr, *name = nullptr;
	GetParam(store, "name", name);
	GetParam(store, "src", src);
	if (!strcmp(GetArchiveName(store), "behavier"))
	{
		if (src)
		{
			return LoadBehavierFbx(name, src);
		}

		const char* arma = nullptr;
		GetParam(store, "armature", arma);
		if (arma)
		{
			auto actions = GetFirstChildArchive(store,"behavier.actions");
			if (actions)
			{
				list<std::pair<string, string>> actionlist;
				for (auto action = GetFirstChildArchive(actions,"action"); action != nullptr; action = GetNextSiblingArchive(action,"action"))
				{
					const char* asrc = nullptr, *aname = nullptr;
					GetParam(action, "name", aname);
					GetParam(action, "src", asrc);
					if (aname && asrc)
						actionlist.emplace_back(aname, asrc);
				}
				return LoadBehavierFbxs(name, arma, actionlist);
			}
		}
		//else
		//{
		//	auto& behavier = assets.AddBehavier(node->Attribute("name"), new behavier_type);
		//	attr = node->Attribute("armature");
		//	if (attr[0] == '{')
		//	{
		//		string key(attr + 1, attr + strlen(attr) - 1);
		//		auto& armature = assets.GetAsset<AssetDictionary::armature_type>(key);
		//		behavier.SetArmature(armature);
		//	}

		//	auto clip = node->FirstChildElement("animation_clip");
		//	while (clip)
		//	{
		//		auto& aniClip = ParseAnimation(assets, clip);
		//		aniClip.SetArmature(behavier.Armature());
		//		behavier.AddAnimationClip(clip->Attribute("name"), move(aniClip));
		//		clip = node->NextSiblingElement("animation_clip");
		//	}
		//	return behavier;
		//}
	}
	return nullptr;
}

inline std::wstring towstring(const string& str)
{
	return std::wstring(str.begin(), str.end());
}

AssetDictionary::mesh_type * AssetDictionary::LoadObjMesh(const string & key, const string & fileName)
{
	typedef DefaultStaticModel mesh_type;
	auto *pObjModel = DefaultStaticModel::CreateFromObjFile((mesh_directory / fileName).wstring(), m_device.Get(), texture_directory.wstring());
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

	auto pModel = model_type::CreateFromData(&mesh, texture_directory.wstring(), m_device.Get());
	//auto pModel = model_type::CreateCube(render_device,1.0f);

	pModel->SetName(fileName);

	for (auto& part : pModel->Parts)
	{
		if (!part.pEffect)
			part.pEffect = default_skinned_effect;

		if (pMaterial)
			part.pMaterial = pMaterial;

		if (part.pMaterial)
			part.pMaterial->SetupEffect(part.pEffect.get());

		part.pMesh->pInputLayout = GetInputLayout<model_type::VertexType>(part.pEffect.get());
	}

	meshes[key] = pModel;
	return meshes[key];
}

AssetDictionary::mesh_type * AssetDictionary::LoadFbxMesh(const string & key, const string & fileName, bool importMaterial)
{
	typedef DefaultSkinningModel model_type;

	FbxParser fbx;
	fbx.ImportMesh((mesh_directory / fileName).string(), true); // with rewind
	auto datas = fbx.GetMeshs();
	if (datas.size() == 0)
		return nullptr;

	auto pModel = model_type::CreateFromDatas(datas, texture_directory.wstring(), nullptr);

	pModel->SetName(fileName);

	if (importMaterial)
	{
		for (auto& part : pModel->Parts)
		{
			auto& pMat = part.pMaterial;
			if (!pMat)
				pMat = default_material;
			else
			{
				auto pPhong = dynamic_cast<PhongMaterial*>(pMat.get());
				if (materials[pPhong->Name] != nullptr)
				{
					pMat = materials[pPhong->Name];
				}
			}
		}
	}

	pModel->CreateDeviceResource(m_device.Get());

	int i = 0;
	for (auto& part : pModel->Parts)
	{
		if (!part.pEffect)
			part.pEffect = default_skinned_effect;

		if (!part.pMaterial)
			part.pMaterial->SetupEffect(part.pEffect.get());

		part.pMesh->pInputLayout = GetInputLayout<model_type::VertexType>(part.pEffect.get());
		i++;
	}

	meshes[key] = pModel;
	return meshes[key];
}

AssetDictionary::texture_type * AssetDictionary::LoadTexture(const string & key, const string & fileName)
{
	textures[key] = DirectX::Texture::CreateFromDDSFile(m_device.Get(), (texture_directory / fileName).c_str());
	return textures[key];
}

AssetDictionary::armature_type * AssetDictionary::LoadArmature(const string & key, const string & fileName)
{
	std::ifstream file((mesh_directory / fileName).wstring());
	auto pArmature = new armature_type(file);
	m_assets[key] = pArmature;
	return pArmature;
}

AssetDictionary::animation_clip_type* AssetDictionary::LoadAnimation(const string & key, const string & fileName)
{
	using clip_type = AssetDictionary::animation_clip_type;
	std::ifstream stream((animation_directory / fileName).wstring());
	if (stream.is_open())
	{
		animations.emplace(key, new clip_type(stream));
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
	auto result = fbxparser.ImportArmature((mesh_directory / armature).string());
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

//task<AssetDictionary::mesh_type*>& AssetDictionary::LoadMeshAsync(const string & key, const string & fileName)
//{
//	using namespace std::placeholders;
//	task<mesh_type*> load_mesh([this, key,fileName] () 
//	{ 
//		return this->LoadObjMesh(key,fileName);
//	});
//	loading_meshes[fileName] = std::move(load_mesh);
//	return loading_meshes[fileName];
//}
//
//task<AssetDictionary::texture_type*>& AssetDictionary::LoadTextureAsync(const string & key, const string & fileName)
//{
//	task<texture_type*> loading_texture([this,key, fileName]() { return this->LoadTexture(key,fileName); });
//	loading_textures[key] = std::move(loading_texture);
//	return loading_textures[key];
//}

AssetDictionary::audio_clip_type* AssetDictionary::GetAudio(const string & key)
{
	return nullptr;// GetAsset<audio_clip_type>(key);
}

void AssetDictionary::SetRenderDevice(IRenderDevice * device)
{
	m_device = device;
	if (!effect_factory)
	{
		auto up_factory = std::make_unique<DirectX::EffectFactory>(device);
		up_factory->SetDirectory(texture_directory.c_str());
		effect_factory = move(up_factory);

		//auto pBEffect = std::make_shared<DirectX::BasicEffect>(device.Get());
		//pBEffect->SetVertexColorEnabled(false);
		//pBEffect->SetTextureEnabled(true);
		//pBEffect->EnableDefaultLighting();
		//default_effect = pBEffect;

		auto pMainEffect = std::make_shared<DirectX::ShadowMapEffect>(device);
		pMainEffect->SetWeightsPerVertex(0); // Disable Skinning
		pMainEffect->SetLightEnabled(0, true);
		pMainEffect->SetLightView(0,XMMatrixLookToRH(XMVectorSet(0, 5, 0, 1), XMVectorSet(0, -1, 0, 0), XMVectorSet(0, 0, -1,0)));
		pMainEffect->SetLightProjection(0, XMMatrixOrthographicRH(5,5,0.01f,10.0f));
		default_effect = pMainEffect;

		default_skinned_effect = default_effect;

		default_envirument_effect = std::make_shared<DirectX::EnvironmentMapEffect>(device);

		//auto pSEffect = std::make_shared<DirectX::DGSLEffect>(device.Get(),nullptr,true);
		////auto pSEffect = std::make_shared<DirectX::SkinnedEffect>(device.Get());
		//pSEffect->SetWeightsPerVertex(4U);
		//pSEffect->SetVertexColorEnabled(true);
		//pSEffect->SetTextureEnabled(false);
		//pSEffect->EnableDefaultLighting();
		//pSEffect->SetAlphaDiscardEnable(true);
		//default_skinned_effect = pSEffect;

		default_material = std::make_unique<PhongMaterial>();
		default_material->pDefaultRequestEffect = default_effect.get();
		default_material->Name = "Default";

		effects["default"] = default_effect.get();
		effects["default_skinned"] = default_skinned_effect.get();
		effects["default_environment"] = default_envirument_effect.get();
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
