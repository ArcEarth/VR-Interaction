#define NOMINMAX
#include "Model.h"
#include <string>
#include "stride_iterator.h"
#include <sstream>
#include <WICTextureLoader.h>
#include <DDSTextureLoader.h>
#include <boost\filesystem.hpp>
#include "..\Extern\tiny_obj_loader.h"
#include <boost\filesystem.hpp>
#include <CommonStates.h>

using namespace DirectX::Scene;
using namespace DirectX;
using namespace std;
using namespace boost::filesystem;

void DirectX::Scene::Mesh::Draw(ID3D11DeviceContext *pContext) const
{
	if (pInputLayout)
		pContext->IASetInputLayout(pInputLayout.Get());

	// Set the input assembler stage
	auto vb = pVertexBuffer.Get();
	UINT vbStride = VertexStride;
	UINT vbOffset = 0;

	pContext->IASetVertexBuffers(0, 1, &vb, &vbStride, &vbOffset);


	//if (pEffect)
	//	pEffect->Apply(pContext);

	pContext->IASetPrimitiveTopology(PrimitiveType);

	if (pIndexBuffer)
	{
		pContext->IASetIndexBuffer(pIndexBuffer.Get(), IndexFormat, 0);
		pContext->DrawIndexed(IndexCount, StartIndex, VertexOffset);
	}
	else
	{
		pContext->Draw(VertexCount, VertexOffset);
	}
	return;
}

DirectX::Scene::GeometryModel::~GeometryModel()
{}

std::shared_ptr<GeometryModel> DirectX::Scene::GeometryModel::CreateFromObjFile(ID3D11Device * pDevice, const std::wstring & file, const std::wstring & textureDir)
{
	return std::make_shared<GeometryModel>(pDevice, file, textureDir);
}

DirectX::Scene::GeometryModel::GeometryModel(ID3D11Device *pDevice, const std::wstring &fileName, const std::wstring& textureDir)
{
	typedef VertexPositionNormalTexture VertexType;
	using namespace tinyobj;
	vector<shape_t> shapes;
	vector<material_t> materis;
	path file(fileName);
	Name = file.filename().replace_extension().string();
	auto dir = file.parent_path();
	auto result = tinyobj::LoadObj(shapes, materis, file.string().c_str(), (dir.string() + "\\").c_str());

	std::vector<std::shared_ptr<Mesh>> Meshs;
	std::vector<std::shared_ptr<PhongMaterial>> Materials;

	//ComPtr<ID3D11DeviceContext> pContext;

	boost::filesystem::path lookup(textureDir);

	//pDevice->GetImmediateContext(&pContext);

	int vOffset = 0, iOffset = 0;

	for (auto& shape : shapes)
	{
		auto N = shape.mesh.positions.size() / 3;

		for (size_t i = 0; i < shape.mesh.indices.size() / 3; i++)
		{
			const auto& idcs = shape.mesh.indices;
			//FacetPrimitives::Triangle<uint16_t> tri{ idcs[i * 3 + 0],idcs[i * 3 + 1],idcs[i * 3 + 2] };
			FacetPrimitives::Triangle<uint16_t> tri{ idcs[i * 3+2],idcs[i * 3 + 1],idcs[i * 3 + 0] };
			Facets.push_back(tri);
		}

		stride_range<Vector3> Pos(reinterpret_cast<Vector3*>(&shape.mesh.positions[0]), sizeof(float) * 3, N);
		if (shape.mesh.normals.size() == 0)
		{
			// Generate smooth vertex normal, should be improved to corperate co-tanget weight
			shape.mesh.normals.resize(N * 3);
			XMVECTOR n, v0, v1, v2;
			stride_range<Vector3> normals(reinterpret_cast<Vector3*>(&shape.mesh.normals[0]), sizeof(float) * 3, N);
			stride_range<FacetPrimitives::Triangle<unsigned int>> facets(reinterpret_cast<FacetPrimitives::Triangle<unsigned int>*>(&shape.mesh.indices[0]), sizeof(unsigned int) * 3, shape.mesh.indices.size() / 3);
			std::fill_n(shape.mesh.normals.begin(), N*3, .0f);
			for (const auto& face : facets)
			{
				v0 = Pos[face.V0];
				v1 = Pos[face.V1];
				v2 = Pos[face.V2];
				v1 -= v0;
				v2 -= v0;
				n = XMVector3Cross(v1, v2);
				n = XMVector3Normalize(n);
				n = XMVectorNegate(n);
				//if (XMVectorGetY(n) < 0.0f)
				//	n = XMVectorNegate(n);
				//facetNormals.emplace_back(n);
				normals[face.V0] += n;
				normals[face.V1] += n;
				normals[face.V2] += n;
			}
			
			for (auto& nor : normals)
			{
				nor.Normalize();
			}
		}
		stride_range<Vector3> Nor(reinterpret_cast<Vector3*>(&shape.mesh.normals[0]), sizeof(float) * 3, N);
		if (shape.mesh.texcoords.size() != 0)
		{
			stride_range<Vector2> Tex(reinterpret_cast<Vector2*>(&shape.mesh.texcoords[0]), sizeof(float) * 2, N);
			for (size_t i = 0; i < N; i++)
			{
				Vertices.emplace_back(Pos[i], Nor[i], Tex[i]);
			}
		}
		else
		{
			for (size_t i = 0; i < N; i++)
			{
				Vertices.emplace_back(Pos[i], Nor[i], Vector2(0,0));
			}
		}


		auto mesh = std::make_shared<Mesh>();
		Parts.emplace_back();
		Parts.back().Name = shape.name;
		Parts.back().pMesh = mesh;

		auto& part = Parts.back();
		auto& box = Parts.back().BoundBox;
		BoundingBox::CreateFromPoints(box, N, (XMFLOAT3*) shape.mesh.positions.data(), sizeof(float) * 3);
		float scale = std::max(box.Extents.x, std::max(box.Extents.y, box.Extents.z));
		for (auto& p : shape.mesh.positions)
		{
			p /= scale;
		}
		BoundingOrientedBox::CreateFromPoints(part.BoundOrientedBox, N, (XMFLOAT3*) shape.mesh.positions.data(), sizeof(float) * 3);
		XMStoreFloat3(&part.BoundOrientedBox.Center, XMLoadFloat3(&part.BoundOrientedBox.Center) * scale);
		XMStoreFloat3(&part.BoundOrientedBox.Extents, XMLoadFloat3(&part.BoundOrientedBox.Extents) * scale);
		for (auto& p : shape.mesh.positions)
		{
			p *= scale;
		}
		mesh->VertexCount = N;
		mesh->IndexCount = shape.mesh.indices.size();
		mesh->PrimitiveType = D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST;
		mesh->IndexFormat = ExtractDXGIFormat<uint16_t>::value;
		mesh->VertexStride = sizeof(VertexPositionNormalTexture);
		mesh->VertexOffset = vOffset;
		mesh->StartIndex = iOffset;
		vOffset += mesh->VertexCount;
		iOffset += mesh->IndexCount;
	}

	Positions = stride_range<Vector3>((Vector3*) &Vertices[0].position, sizeof(VertexType), Vertices.size());
	Normals = stride_range<Vector3>((Vector3*) &Vertices[0].normal, sizeof(VertexType), Vertices.size());
	TexCoords = stride_range<Vector2>((Vector2*) &Vertices[0].textureCoordinate, sizeof(VertexType), Vertices.size());

	BoundingBox::CreateFromPoints(BoundBox, Positions.size(), &Positions[0], sizeof(VertexType));

	float scale = std::max(BoundBox.Extents.x,std::max(BoundBox.Extents.y, BoundBox.Extents.z));
	XMVECTOR s = XMVectorReplicate(scale);
	for (auto& p : Positions)
	{
		p = (XMVECTOR)p / s;
	}
	BoundingOrientedBox::CreateFromPoints(BoundOrientedBox, Positions.size(), &Positions[0], sizeof(VertexType));
	BoundingSphere::CreateFromPoints(BoundSphere, Positions.size(), &Positions[0], sizeof(VertexType));
	for (auto& p : Positions)
	{
		p = (XMVECTOR)p * s;
	}
	XMStoreFloat3(&BoundOrientedBox.Center, XMLoadFloat3(&BoundOrientedBox.Center) * s);
	XMStoreFloat3(&BoundOrientedBox.Extents, XMLoadFloat3(&BoundOrientedBox.Extents) * s);
	XMStoreFloat3(&BoundSphere.Center, XMLoadFloat3(&BoundSphere.Center) * s);
	BoundSphere.Radius *= scale;

	// Device Dependent Resources Creation
	auto pVertexBuffer = DirectX::CreateVertexBuffer(pDevice, Vertices.size(), &Vertices[0]);
	auto pIndexBuffer = DirectX::CreateIndexBuffer(pDevice, Facets.size() * 3, &Facets[0].V0);

	for (auto& mat : materis)
	{
		HRESULT hr = S_OK;
		ComPtr<ID3D11Resource> pResource;
		auto pMaterial = make_shared<PhongMaterial>();
		pMaterial->Name = mat.name;
		pMaterial->Alpha = mat.dissolve;
		pMaterial->DiffuseColor = Color(mat.diffuse);
		pMaterial->AmbientColor = Color(mat.ambient);
		pMaterial->SpecularColor = Color(mat.specular);
		if (!mat.diffuse_texname.empty())
		{
			auto fileName = lookup / mat.diffuse_texname;
			if (fileName.extension() != "dds")
			hr = CreateWICTextureFromFile(pDevice, fileName.wstring().data(), &pResource, &pMaterial->DiffuseMap);
		}
		if (!mat.specular_texname.empty())
		{
			auto fileName = lookup / mat.specular_texname;
			hr = CreateWICTextureFromFile(pDevice, fileName.wstring().data(), &pResource, &pMaterial->SpecularMap);

		}
		if (!mat.normal_texname.empty())
		{
			auto fileName = lookup / mat.normal_texname;
			hr = CreateWICTextureFromFile(pDevice, fileName.wstring().data(), &pResource, &pMaterial->NormalMap);
		}
		Materials.push_back(pMaterial);
	}

	for (size_t i = 0; i < shapes.size(); i++)
	{
		const auto& shape = shapes[i];
		auto &part = Parts[i];
		if (shape.mesh.material_ids[0] >= 0)
			part.pMaterial = Materials[shape.mesh.material_ids[0]];
		else
			part.pMaterial = nullptr;
		part.pMesh->pVertexBuffer = pVertexBuffer;
		part.pMesh->pIndexBuffer = pIndexBuffer;
	}

	//std::ifstream stream;
	//stream.open(file);

	//if (!stream.is_open())
	//	throw;
	//string tag;
	//Vector3 vec3;
	//Vector2 vec2;
	//Color color;
	//FacetPrimitives::Triangle<IndexType> tri;

	//std::vector<Vector3> positions;
	//std::vector<Vector3> normals;
	//std::vector<Vector2> texcoords;
	//string line;
	//color.w = 1.0f;
	//while (!stream.eof())
	//{
	//	std::getline(stream, line);
	//	stringstream lss(line);

	//	lss >> tag;
	//	if (tag == "mtllib")
	//	{ 
	//	}
	//	else if (tag == "usemtl")
	//	{
	//	}
	//	else if (tag == "o")
	//	{
	//	}
	//	else if (tag == "g")
	//	{
	//	}
	//	else if (tag == "v") // (x,y,z[,w]) coordinates, w is optional and defaults to 1.0.
	//	{
	//		lss >> vec3.x >> vec3.y >> vec3.z;
	//		positions.push_back(vec3);
	//	}
	//	else if (tag == "vt") // in (u, v [,w]), these will vary between 0 and 1, w is optional and defaults to 0.
	//	{
	//		lss >> vec2.x >> vec2.y;
	//		texcoords.push_back(vec2);
	//	}
	//	else if (tag == "vn") // (u, v [,w]) normals might not be unit. 
	//	{
	//		lss >> vec3.x >> vec3.y >> vec3.z;
	//		normals.push_back(vec3);
	//	}
	//	else if (tag == "vp") // ( u [,v] [,w] ) free form geometry statement
	//	{
	//		lss >> vec3.x >> vec3.y >> vec3.z; //Ignore the unknow parameter data
	//											  //VertexParameters.push_back(vec3);
	//	}
	//	else if (tag == "f")
	//	{
	//		lss >> tri.V0 >> tri.V1 >> tri.V2; // f v1 v2 v3 ...
	//											  // f v1/vt1/vn1 v2/vt2/vn2 v3/vt3/vn3 ...
	//											  // f v1//vn1 v2//vn2 v3//vn3 ...
	//		tri.V0--, tri.V1--, tri.V2--; // The obj file format starts the indices from 1, but as usual, it should starts from 0
	//		Facets.push_back(tri);
	//	}
	//}

	//auto N = positions.size();
	//assert(normals.size() == 0 || N == normals.size());
	//assert(texcoords.size() == 0 || N == texcoords.size());

	//BoundingBox box;
	//BoundingBox::CreateFromPoints(box, positions.size(), &positions[0], sizeof(Vector3));
	//BoundingOrientedBox::CreateFromBoundingBox(Bound, box);
	//Vertices.resize(N);

	//Positions = stride_range<Vector3>((Vector3*) &Vertices[0].position, sizeof(VertexType), Vertices.size());
	//std::copy_n(positions.begin(), N, Positions.begin());

	//Normals = stride_range<Vector3>((Vector3*) &Vertices[0].normal, sizeof(VertexType), Vertices.size());
	//if (normals.size() == 0)
	//{
	//	GenerateNormal();
	//}
	//else
	//{
	//	std::copy_n(normals.begin(), N, Normals.begin());
	//}

	//if (texcoords.size() > 0)
	//{
	//	TexCoords = stride_range<Vector2>((Vector2*) &Vertices[0].textureCoordinate, sizeof(VertexType), Vertices.size());
	//	std::copy_n(texcoords.begin(), N, TexCoords.begin());
	//}
}

//void DirectX::Scene::ObjMesh::GenerateNormal()
//{
//	auto N = Vertices.size();
//	//std::vector<Vector3> facetNormals;
//	XMVECTOR n, v0, v1, v2;
//	std::fill_n(Normals.begin(), N, Vector3());
//	for (const auto& face : Facets)
//	{
//		v0 = Positions[face.V0];
//		v1 = Positions[face.V1];
//		v2 = Positions[face.V2];
//		v1 -= v0;
//		v2 -= v0;
//		n = XMVector3Cross(v1, v2);
//		n = XMVector3Normalize(n);
//		if (XMVectorGetY(n) < 0.0f)
//			n = XMVectorNegate(n);
//		//facetNormals.emplace_back(n);
//		Normals[face.V0] += n;
//		Normals[face.V1] += n;
//		Normals[face.V2] += n;
//	}
//
//	for (auto& nor : Normals)
//	{
//		nor.Normalize();
//	}
//}


//
//void DirectX::Scene::PhongMaterial::LoadFromFile(ID3D11Device* pDevice, const std::wstring &file, const std::wstring &dir)
//{
//	std::ifstream stream;
//	stream.open(file);
//
//	if (!stream.is_open())
//		throw;
//
//	string tag;
//	string line;
//	ComPtr<ID3D11DeviceContext> pContext;
//	ComPtr<ID3D11Resource> pResource;
//
//	boost::filesystem::path lookup(dir);
//
//	pDevice->GetImmediateContext(&pContext);
//
//	while (!stream.eof())
//	{
//		std::getline(stream, line);
//		stringstream lss(line);
//		lss >> tag;
//		if (tag == "#")
//		{
//		}
//		else if (tag == "Ka")
//		{ 
//			lss >> AmbientColor.x >> AmbientColor.y >> AmbientColor.z;// >> AmbientColor.w;
//		}
//		else if (tag == "Kd")
//		{
//			lss >> DiffuseColor.x >> DiffuseColor.y >> DiffuseColor.z;// >> DiffuseColor.w;
//		}
//		else if (tag == "Ks")
//		{
//			lss >> SpecularColor.x >> SpecularColor.y >> SpecularColor.z;// >> DiffuseColor.w;
//		}
//		else if (tag == "d" || tag == "Tr")
//		{
//			lss >> Alpha;
//		}
//		else if (tag == "illum")
//		{
//
//		}
//		else if (tag == "map_Ka")
//		{
//
//		}
//		else if (tag == "map_Kd")
//		{
//			lss >> tag;
//			wstring fileName(tag.begin(), tag.end());
//			ThrowIfFailed(
//				CreateWICTextureFromFile(pDevice, pContext.Get(), fileName.data(), &pResource, &DiffuseMap));
//		}
//		else if (tag == "map_Ks")
//		{
//
//		}
//		else if (tag == "map_Ns")
//		{
//
//		}
//		else if (tag == "map_d")
//		{
//
//		}
//		else if (tag == "map_bump" || tag == "bump")
//		{
//			lss >> tag;
//			auto fileName = (lookup / tag).wstring();
//			ThrowIfFailed(
//				CreateWICTextureFromFile(pDevice, pContext.Get(), fileName.data(), &pResource, &BumpMap));
//		}
//		else if (tag == "map_disp" || tag == "disp")
//		{
//			lss >> tag;
//			auto fileName = (lookup / tag).wstring();
//			ThrowIfFailed(
//				CreateWICTextureFromFile(pDevice, pContext.Get(), fileName.data(), &pResource, &DisplaceMap));
//		}
//		else if (tag == "map_decal" || tag == "decal")
//		{
//
//		}
//	}
//}

//using namespace std::tr2;
//DirectX::Scene::ObjModel::ObjModel(ID3D11Device * pDevice, const std::wstring & file, const std::wstring &textureDirectory, const std::shared_ptr<IEffect>& pEffect)
//	: ObjMesh(file)
//{
//	auto path = boost::filesystem::path(file);
//	path.replace_extension(".mtl");
//	PhongMaterial::LoadFromFile(pDevice,path.wstring(),textureDirectory);
//	ObjMesh::Update(pDevice);
//}

void DirectX::Scene::ModelCollection::AddModel(const std::shared_ptr<ModelNode>& pModel)
{
	pModel->pParent = this;
	Children.push_back(pModel);
}

void DirectX::Scene::ModelCollection::Render(ID3D11DeviceContext * pContext, IEffect * pEffect)
{
	int count = Children.size();
	for (size_t i = 0; i < count; i++)
	{
		auto& model = Children[i];
		model->Render(pContext, pEffect);
	}
}

XMMATRIX DirectX::Scene::ModelNode::GetModelMatrix() const
{
	if (pParent)
		return pParent->GetModelMatrix() * RigidObject::GetModelMatrix();
	else
		return RigidObject::GetModelMatrix();
}

// Transformed OrientedBounding Box

BoundingOrientedBox DirectX::Scene::ModelNode::GetOrientedBoundingBox() const
{
	BoundingOrientedBox box;
	BoundOrientedBox.Transform(box, GetModelMatrix());
	return box;
}

// Transformed Bounding Box

BoundingBox DirectX::Scene::ModelNode::GetBoundingBox() const {
	BoundingBox box;
	BoundBox.Transform(box, GetModelMatrix());
	return box;
}

// Transformed Bounding Sphere

BoundingSphere DirectX::Scene::ModelNode::GetBoundingSphere() const
{
	BoundingSphere sphere;
	BoundSphere.Transform(sphere, GetModelMatrix());
	return sphere;
}

void DirectX::Scene::ModelPart::Render(ID3D11DeviceContext * pContext, IEffect * pEffect)
{
	if (pEffect == nullptr)
	{
		pMesh->Draw(pContext);
	}
	else
	{
		auto pMEffect = dynamic_cast<BasicEffect*>(pEffect);
		if (pMEffect && pMaterial)
		{
			pMEffect->SetAlpha(pMaterial->GetAlpha());
			if (pMaterial->GetDiffuseMap())
			{
				pMEffect->SetTextureEnabled(true);
				pMEffect->SetTexture(pMaterial->GetDiffuseMap());
				pMEffect->SetDiffuseColor(pMaterial->GetDiffuseColor());
			}
			else
			{
				pMEffect->SetTextureEnabled(false);
				pMEffect->SetDiffuseColor(pMaterial->GetDiffuseColor());
			}
			pMEffect->SetSpecularColor(pMaterial->GetSpecularColor());
		}
		pEffect->Apply(pContext);
		pMesh->Draw(pContext);
	}
}

void DirectX::Scene::Model::Render(ID3D11DeviceContext * pContext, IEffect* pEffect)
{
	auto world = this->GetModelMatrix();
	for (auto& part : Parts)
	{
		auto pEffectM = dynamic_cast<IEffectMatrices*>(pEffect);
		if (pEffectM)
		{
			pEffectM->SetWorld(world);
		} 
		part.Render(pContext,pEffect);
	}
}
