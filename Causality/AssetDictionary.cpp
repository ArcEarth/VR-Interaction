#include "AssetDictionary.h"
using namespace Causality;

AssetDictionary::AssetDictionary()
{
}


AssetDictionary::~AssetDictionary()
{
}

inline std::wstring towstring(const string& str)
{
	return std::wstring(str.begin(), str.end());
}

AssetDictionary::mesh_type & Causality::AssetDictionary::LoadMesh(const string & fileName)
{
	mesh_type::CreateFromObjFile(&meshes[fileName], render_device, towstring(fileName), texure_directory.wstring());
	return meshes[fileName];
}

AssetDictionary::texture_type & Causality::AssetDictionary::LoadTexture(const string & fileName)
{
	textures[fileName] = DirectX::Texture::CreateFromDDSFile(render_device, towstring(fileName).c_str());
	return textures[fileName];
}

AssetDictionary::armature_type & Causality::AssetDictionary::LoadArmature(const string & fileName)
{
	std::ifstream file(fileName);
	auto pArmature = new armature_type(file);
	assets[fileName] = pArmature;
	return *pArmature;
}

task<AssetDictionary::mesh_type*>& Causality::AssetDictionary::LoadMeshAsync(const string & fileName)
{
	using namespace std::placeholders;
	task<mesh_type*> load_mesh([this,fileName] () { return &this->LoadMesh(fileName); });
	loading_meshes[fileName] = std::move(load_mesh);
	return loading_meshes[fileName];
}

task<AssetDictionary::texture_type*>& Causality::AssetDictionary::LoadTextureAsync(const string & fileName)
{
	task<texture_type*> loading_texture([this, fileName]() { return &this->LoadTexture(fileName); });
	loading_textures[fileName] = std::move(loading_texture);
	return loading_textures[fileName];
}
