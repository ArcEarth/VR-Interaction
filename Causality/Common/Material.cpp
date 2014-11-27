#include <fstream>
#include <WICTextureLoader.h>
#include "Material.h"
#include "../Extern/tiny_obj_loader.h"
#include <boost\filesystem.hpp>
using namespace DirectX;
using namespace DirectX::Scene;
using namespace tinyobj;
using namespace std;
using namespace Microsoft::WRL;

DirectX::Scene::PhongMaterial::PhongMaterial()
{}

std::vector<std::shared_ptr<PhongMaterial>> DirectX::Scene::PhongMaterial::CreateFromMtlFile(ID3D11Device * pDevice, const std::wstring & file, const std::wstring & lookupDirectory)
{
	std::vector<std::shared_ptr<PhongMaterial>> Materials;
	std::map<std::string, int> map;
	std::vector<material_t> materis;
	std::ifstream fin(file);
	tinyobj::LoadMtl(map, materis, fin);
	ComPtr<ID3D11DeviceContext> pContext;
	ComPtr<ID3D11Resource> pResource;
	boost::filesystem::path lookup(lookupDirectory);

	pDevice->GetImmediateContext(&pContext);

	for (auto& mat : materis)
	{
		auto pMaterial = make_shared<PhongMaterial>();
		pMaterial->Name = mat.name;
		pMaterial->Alpha = mat.dissolve;
		pMaterial->DiffuseColor = Color(mat.diffuse);
		pMaterial->AmbientColor = Color(mat.ambient);
		pMaterial->SpecularColor = Color(mat.specular);
		if (!mat.diffuse_texname.empty())
		{
			auto fileName = lookup / mat.diffuse_texname;
			CreateWICTextureFromFile(pDevice, pContext.Get(), fileName.wstring().data(), &pResource, &pMaterial->DiffuseMap);
		}
		if (!mat.specular_texname.empty())
		{
			auto fileName = lookup / mat.specular_texname;
			CreateWICTextureFromFile(pDevice, pContext.Get(), fileName.wstring().data(), &pResource, &pMaterial->SpecularMap);
		}
		if (!mat.normal_texname.empty())
		{
			auto fileName = lookup / mat.normal_texname;
			CreateWICTextureFromFile(pDevice, pContext.Get(), fileName.wstring().data(), &pResource, &pMaterial->NormalMap);
		}
		Materials.push_back(pMaterial);
	}
	return Materials;
}

Color DirectX::Scene::PhongMaterial::GetAmbientColor() const
{
	return AmbientColor;
}

Color DirectX::Scene::PhongMaterial::GetDiffuseColor() const
{
	return DiffuseColor;
}

Color DirectX::Scene::PhongMaterial::GetSpecularColor() const
{
	return SpecularColor;
}

float DirectX::Scene::PhongMaterial::GetAlpha() const
{
	return Alpha;
}

ID3D11ShaderResourceView * DirectX::Scene::PhongMaterial::GetDiffuseMap() const
{
	return DiffuseMap.Get();
}

ID3D11ShaderResourceView * DirectX::Scene::PhongMaterial::GetNormalMap() const
{
	return NormalMap.Get();
}

ID3D11ShaderResourceView * DirectX::Scene::PhongMaterial::GetSpecularMap() const
{
	return nullptr;
}

ID3D11ShaderResourceView * DirectX::Scene::PhongMaterial::GetDisplaceMap() const
{
	return DisplaceMap.Get();
}