#include "pch_directX.h"
#define NOMINMAX
#include "Model.h"
#include <string>
#include "stride_iterator.h"
#include <sstream>
#include <WICTextureLoader.h>
#include <DDSTextureLoader.h>
#include <boost\filesystem.hpp>
#include "Extern\tiny_obj_loader.h"
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

bool DirectX::Scene::GeometryModel::CreateFromObjFile(DirectX::Scene::GeometryModel *pResult, ID3D11Device * pDevice, const std::wstring & fileName, const std::wstring & textureDir)
{
	typedef VertexPositionNormalTexture VertexType;
	using namespace tinyobj;
	vector<shape_t> shapes;
	vector<material_t> materis;
	path file(fileName);
	pResult->Name = file.filename().replace_extension().string();
	auto dir = file.parent_path();
	auto result = tinyobj::LoadObj(shapes, materis, file.string().c_str(), (dir.string() + "\\").c_str());

	std::vector<std::shared_ptr<Mesh>> Meshs;
	std::vector<std::shared_ptr<PhongMaterial>> Materials;

	boost::filesystem::path lookup(textureDir);

	auto& Vertices = pResult->Vertices;
	auto& Facets = pResult->Facets;
	auto& Parts = pResult->Parts;
	auto& Positions = pResult->Positions;
	auto& Normals = pResult->Normals;
	auto& TexCoords = pResult->TexCoords;

	int vOffset = 0, iOffset = 0;

	for (auto& shape : shapes)
	{
		auto N = shape.mesh.positions.size() / 3;

		for (size_t i = 0; i < shape.mesh.indices.size() / 3; i++)
		{
			const auto& idcs = shape.mesh.indices;
			//FacetPrimitives::Triangle<uint16_t> tri{ idcs[i * 3 + 0],idcs[i * 3 + 1],idcs[i * 3 + 2] };
			FacetPrimitives::Triangle<uint16_t> tri{ idcs[i * 3 + 2],idcs[i * 3 + 1],idcs[i * 3 + 0] };
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
			std::fill_n(shape.mesh.normals.begin(), N * 3, .0f);
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
				Vertices.emplace_back(Pos[i], Nor[i], Vector2(0, 0));
			}
		}


		auto mesh = std::make_shared<Mesh>();
		Parts.emplace_back(new ModelPart);
		Parts.back()->Name = shape.name;
		Parts.back()->pMesh = mesh;

		auto& part = Parts.back();
		auto& box = Parts.back()->BoundBox;
		BoundingBox::CreateFromPoints(box, N, (XMFLOAT3*) shape.mesh.positions.data(), sizeof(float) * 3);
		float scale = std::max(box.Extents.x, std::max(box.Extents.y, box.Extents.z));
		for (auto& p : shape.mesh.positions)
		{
			p /= scale;
		}

		//BoundingOrientedBox::CreateFromPoints(part.BoundOrientedBox, N, (XMFLOAT3*) shape.mesh.positions.data(), sizeof(float) * 3);
		CreateBoundingOrientedBoxFromPoints(part->BoundOrientedBox, N, (XMFLOAT3*) shape.mesh.positions.data(), sizeof(float) * 3);
		XMStoreFloat3(&part->BoundOrientedBox.Center, XMLoadFloat3(&part->BoundOrientedBox.Center) * scale);
		XMStoreFloat3(&part->BoundOrientedBox.Extents, XMLoadFloat3(&part->BoundOrientedBox.Extents) * scale);
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

	BoundingBox::CreateFromPoints(pResult->BoundBox, Positions.size(), &Positions[0], sizeof(VertexType));

	float scale = std::max(pResult->BoundBox.Extents.x, std::max(pResult->BoundBox.Extents.y, pResult->BoundBox.Extents.z));
	XMVECTOR s = XMVectorReplicate(scale);
	for (auto& p : Positions)
	{
		p = (XMVECTOR) p / s;
	}
	CreateBoundingOrientedBoxFromPoints(pResult->BoundOrientedBox, Positions.size(), &Positions[0], sizeof(VertexType));
	//BoundingOrientedBox::CreateFromPoints(BoundOrientedBox, Positions.size(), &Positions[0], sizeof(VertexType));
	BoundingSphere::CreateFromPoints(pResult->BoundSphere, Positions.size(), &Positions[0], sizeof(VertexType));
	for (auto& p : Positions)
	{
		p = (XMVECTOR) p * s;
	}
	XMStoreFloat3(&pResult->BoundOrientedBox.Center, XMLoadFloat3(&pResult->BoundOrientedBox.Center) * s);
	XMStoreFloat3(&pResult->BoundOrientedBox.Extents, XMLoadFloat3(&pResult->BoundOrientedBox.Extents) * s);
	XMStoreFloat3(&pResult->BoundSphere.Center, XMLoadFloat3(&pResult->BoundSphere.Center) * s);
	pResult->BoundSphere.Radius *= scale;

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
		ThrowIfFailed(hr);
		Materials.push_back(pMaterial);
	}

	for (size_t i = 0; i < shapes.size(); i++)
	{
		const auto& shape = shapes[i];
		auto &part = Parts[i];
		if (shape.mesh.material_ids[0] >= 0)
			part->pMaterial = Materials[shape.mesh.material_ids[0]];
		else
			part->pMaterial = nullptr;
		part->pMesh->pVertexBuffer = pVertexBuffer;
		part->pMesh->pIndexBuffer = pIndexBuffer;
	}
	return true;
}

BasicModel * DirectX::Scene::GeometryModel::ReleaseCpuResource()
{
	Vertices.clear();
	Facets.clear();
	return this;
}

void ModelCollection::push_back(const value_type& _Val)
{
	_Val->pParent = this;
	ContainnerType::push_back(_Val);
}
void ModelCollection::push_back(value_type&& _Val)
{
	_Val->pParent = this;
	ContainnerType::push_back(std::move(_Val));
}

void DirectX::Scene::ModelCollection::Render(ID3D11DeviceContext * pContext, IEffect * pEffect)
{
	int count = size();
	for (size_t i = 0; i < count; i++)
	{
		auto& model = at(i);
		model->Render(pContext, pEffect);
	}
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

void DirectX::Scene::BasicModel::Render(ID3D11DeviceContext * pContext, IEffect* pEffect)
{
	auto world = this->GetWorldMatrix();
	for (auto& part : Parts)
	{
		auto pEffectM = dynamic_cast<IEffectMatrices*>(pEffect);
		if (pEffectM)
		{
			pEffectM->SetWorld(world);
		} 
		part->Render(pContext,pEffect);
	}
}

XMMATRIX IModelNode::GetWorldMatrix() const
{
	if (pParent)
		return pParent->GetWorldMatrix() * GetModelMatrix();
	else
		return GetModelMatrix();
}

IModelNode::~IModelNode() {}

// Transformed OrientedBounding Box
BoundingOrientedBox IModelNode::GetOrientedBoundingBox() const
{
	BoundingOrientedBox box;
	BoundOrientedBox.Transform(box, GetWorldMatrix());
	return box;
}

// Transformed Bounding Box
BoundingBox IModelNode::GetBoundingBox() const {
	BoundingBox box;
	BoundBox.Transform(box, GetWorldMatrix());
	return box;
}

// Transformed Bounding Sphere
BoundingSphere IModelNode::GetBoundingSphere() const
{
	BoundingSphere sphere;
	BoundSphere.Transform(sphere, GetWorldMatrix());
	return sphere;
}

void XM_CALLCONV DirectX::Scene::IModelNode::SetModelMatrix(DirectX::FXMMATRIX model)
{
	XMStoreFloat4x4(&LocalMatrix, model);
}

XMMATRIX DirectX::Scene::IModelNode::GetModelMatrix() const
{
	return XMLoadFloat4x4(&LocalMatrix);
}

//void XM_CALLCONV DirectX::Scene::LocalMatrixHolder::SetModelMatrix(DirectX::FXMMATRIX model)
//{
//	XMStoreFloat4x4(&LocalMatrix,model);
//}
//
//XMMATRIX DirectX::Scene::LocalMatrixHolder::GetModelMatrix() const
//{
//	return XMLoadFloat4x4(&LocalMatrix);
//}
