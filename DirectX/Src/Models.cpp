#include "pch_directX.h"
#define NOMINMAX
#include "Models.h"
#include <string>
#include "stride_range.h"
#include <sstream>
#include <WICTextureLoader.h>
#include <DDSTextureLoader.h>
#include <boost\filesystem.hpp>
#include "tiny_obj_loader.h"
#include <boost\filesystem.hpp>
#include <CommonStates.h>
#include "MeshData.h"
#include <algorithm>
#include <unordered_map>

//using namespace Causality;
using namespace DirectX;
using namespace DirectX::Scene;
using namespace std;
using namespace stdx;
using namespace boost::filesystem;

using Microsoft::WRL::ComPtr;
unordered_map<const D3D11_INPUT_ELEMENT_DESC*, unordered_map<void const*, ComPtr<ID3D11InputLayout>>>
g_InputLayoutLookupTable;

ComPtr<ID3D11InputLayout>& MeshBuffer::LookupInputLayout(const D3D11_INPUT_ELEMENT_DESC* pInputElements, IEffect * pEffect)
{
	void const* shaderByteCode;
	size_t byteCodeLength;
	pEffect->GetVertexShaderBytecode(&shaderByteCode, &byteCodeLength);
	auto& pLayout = g_InputLayoutLookupTable[pInputElements][shaderByteCode];
	return pLayout;
}

void MeshBuffer::CreateInputLayout(ID3D11Device * pDevice, IEffect * pEffect)
{
	assert(pDevice != nullptr);
	assert(pEffect != nullptr);
	assert(pInputElements != nullptr);

	void const* shaderByteCode;
	size_t byteCodeLength;
	pEffect->GetVertexShaderBytecode(&shaderByteCode, &byteCodeLength);
	CreateInputLayout(pDevice, shaderByteCode, byteCodeLength);
}

void MeshBuffer::CreateInputLayout(ID3D11Device * pDevice, const void * shaderByteCode, size_t byteCodeLength)
{
	auto& pLayout = g_InputLayoutLookupTable[pInputElements][shaderByteCode];
	if (pLayout == nullptr)
	{
		ThrowIfFailed(
			pDevice->CreateInputLayout(pInputElements, InputElementCount, shaderByteCode, byteCodeLength, &pInputLayout)
			);
	}
	if (pInputLayout == nullptr)
	{
		pInputLayout = pLayout;
	}
}

void MeshBuffer::Draw(ID3D11DeviceContext *pContext, IEffect *pEffect) const
{
	if (pEffect == nullptr && pInputLayout)
		pContext->IASetInputLayout(pInputLayout.Get());
	else
	{
		void const* shaderByteCode;
		size_t byteCodeLength;
		pEffect->GetVertexShaderBytecode(&shaderByteCode, &byteCodeLength);
		auto& pLayout = g_InputLayoutLookupTable[pInputElements][shaderByteCode];
		if (pLayout == nullptr)
		{
			ComPtr<ID3D11Device> pDevice;
			pContext->GetDevice(&pDevice);
			ThrowIfFailed(
				pDevice->CreateInputLayout(pInputElements, InputElementCount, shaderByteCode, byteCodeLength, &pLayout)
				);
		}
		pContext->IASetInputLayout(pLayout.Get());
	}

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

DefaultStaticModel * DefaultStaticModel::CreateFromObjFile(const std::wstring & fileName, ID3D11Device * pDevice, const std::wstring & textureDir)
{
	typedef VertexPositionNormalTexture VertexType;
	using namespace tinyobj;
	vector<shape_t> shapes;
	vector<material_t> materis;
	path file(fileName);
	auto dir = file.parent_path();
	auto result = tinyobj::LoadObj(shapes, materis, file.string().c_str(), (dir.string() + "\\").c_str());

	if (!result.empty())
		return nullptr; // error happend while loading obj file

	DefaultStaticModel *pResult = new DefaultStaticModel();
	pResult->_Name = file.filename().replace_extension().string();

	std::vector<std::shared_ptr<MeshBuffer>> Meshs;
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
				//n = XMVectorNegate(n);
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


		auto mesh = std::make_shared<MeshBuffer>();
		Parts.emplace_back();
		Parts.back().Name = shape.name;
		Parts.back().pMesh = mesh;

		auto& part = Parts.back();

		DirectX::CreateBoundingBoxesFromPoints(part.BoundBox, part.BoundOrientedBox,
			N, (XMFLOAT3*)shape.mesh.positions.data(), sizeof(XMFLOAT3));

		mesh->SetInputElementDescription<VertexPositionNormalTexture>();
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

	Positions = stride_range<Vector3>((Vector3*)&Vertices[0].position, sizeof(VertexType), Vertices.size());
	Normals = stride_range<Vector3>((Vector3*)&Vertices[0].normal, sizeof(VertexType), Vertices.size());
	TexCoords = stride_range<Vector2>((Vector2*)&Vertices[0].textureCoordinate, sizeof(VertexType), Vertices.size());

	DirectX::CreateBoundingBoxesFromPoints(pResult->BoundBox, pResult->BoundOrientedBox,
		Positions.size(), &Positions[0], sizeof(VertexType));

	BoundingSphere::CreateFromPoints(pResult->BoundSphere, Positions.size(), &Positions[0], sizeof(VertexType));

	for (auto& mat : materis)
	{
		HRESULT hr = S_OK;
		ComPtr<ID3D11Resource> pResource;
		auto pMaterial = make_shared<PhongMaterial>();
		pMaterial->Name = mat.name;
		pMaterial->Alpha = mat.dissolve;
		pMaterial->DiffuseColor = Color(mat.diffuse[0], mat.diffuse[1], mat.diffuse[2], 1.0f);
		pMaterial->AmbientColor = Color(mat.ambient[0], mat.ambient[1], mat.ambient[2], 1.0f);
		pMaterial->SpecularColor = Color(mat.specular[0], mat.specular[1], mat.specular[2], 1.0f);
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

	// Device Dependent Resources Creation
	auto pVertexBuffer = DirectX::CreateVertexBuffer(pDevice, Vertices.size(), &Vertices[0]);
	auto pIndexBuffer = DirectX::CreateIndexBuffer(pDevice, Facets.size() * 3, &Facets[0].V0);

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
	return pResult;;
}

void DefaultStaticModel::Render(ID3D11DeviceContext * pContext, const Matrix4x4 & transform, IEffect * pEffect)
{
	// Disable Skin effect as this is a static model
	auto pSkinning = dynamic_cast<IEffectSkinning*>(pEffect);
	if (pSkinning)
	{
		pSkinning->SetWeightsPerVertex(0);
		//pSkinning->ResetBoneTransforms();
	}
	else if (pEffect == nullptr) // Make sure to disable the skinning effect for default pass
	{
		for (auto& part : Parts)
		{
			pSkinning = dynamic_cast<IEffectSkinning*>(part.pEffect.get());
			if (pSkinning)
			{
				pSkinning->SetWeightsPerVertex(0);
			}
		}
	}
	CompositionModel::Render(pContext, transform, pEffect);
}

bool DefaultStaticModel::IsInMemery() const
{
	return Vertices.size() > 0 && Facets.size() > 0;
}

bool DefaultStaticModel::CreateDeviceResource(ID3D11Device * pDevice)
{

	for (size_t i = 0; i < Parts.size(); i++)
	{
		auto &part = Parts[i];
		part.pMesh->pVertexBuffer.Reset();
		part.pMesh->pIndexBuffer.Reset();
	}

	// Device Dependent Resources Creation
	auto pVertexBuffer = DirectX::CreateVertexBuffer(pDevice, Vertices.size(), &Vertices[0]);
	if (!pVertexBuffer)
		return false;
	auto pIndexBuffer = DirectX::CreateIndexBuffer(pDevice, Facets.size() * 3, &Facets[0].V0);
	if (!pIndexBuffer)
		return false;

	for (size_t i = 0; i < Parts.size(); i++)
	{
		auto &part = Parts[i];
		part.pMesh->pVertexBuffer = pVertexBuffer;
		part.pMesh->pIndexBuffer = pIndexBuffer;
	}
	return true;
}

void DefaultStaticModel::ReleaseDynamicResource()
{
	Vertices.clear();
	Facets.clear();
}

bool DefaultStaticModel::Serialize(std::ostream & binary) const
{
	return false;
}

bool DefaultStaticModel::Deserialize(std::istream & binary)
{
	return false;
}

bool DefaultStaticModel::Reload()
{
	return false;
}

DefaultStaticModel::DefaultStaticModel() {}

DefaultStaticModel::~DefaultStaticModel()
{

}

void CollectionModel::AddChild(const std::shared_ptr<IModelNode>& model, const MatrixTransform & transform)
{
	//model->SetParent(this);
	Children.emplace_back();
	Children.back().Transform = transform;
	Children.back().Model = model;

	BoundingBox::CreateMerged(BoundBox, BoundBox, model->GetBoundingBox());
	//BoundingOrientedBox::CreateFromBoundingBox()
}

void CollectionModel::Render(ID3D11DeviceContext * pContext, const Matrix4x4& transform, IEffect * pEffect)
{
	XMMATRIX world = transform;
	for (auto& child : Children)
	{
		XMMATRIX child_transform = child.Transform;
		child_transform *= world;
		child.Model->Render(pContext, child_transform, pEffect);
	}
}

void ModelPart::Render(ID3D11DeviceContext * pContext, IEffect * pEffect)
{
	if (pEffect == nullptr)
	{
		pEffect = this->pEffect.get(); // pMaterial->GetRequestEffect
	}

	if (pMaterial)
	{
		pMaterial->SetupEffect(pEffect);
	}

	if (pEffect)
		pEffect->Apply(pContext);

	pMesh->Draw(pContext, pEffect);
}

void CompositionModel::Render(ID3D11DeviceContext * pContext, const Matrix4x4& transform, IEffect* pEffect)
{
	XMMATRIX world = transform;
	auto pEffectM = dynamic_cast<IEffectMatrices*>(pEffect);
	for (auto& part : Parts)
	{
		if (pEffect && !pEffectM)
			pEffectM = dynamic_cast<IEffectMatrices*>(part.pEffect.get());

		if (pEffectM)
			pEffectM->SetWorld(world);

		part.Render(pContext, pEffect);
	}
}

//XMMATRIX IModelNode::GetWorldMatrix() const
//{
//	if (pParent)
//		return pParent->GetWorldMatrix() * GetModelMatrix();
//	else
//		return GetModelMatrix();
//}
//
//IModelNode::~IModelNode() {}
//
//// Transformed OrientedBounding Box
//BoundingOrientedBox IModelNode::GetOrientedBoundingBox() const
//{
//	BoundingOrientedBox box;
//	BoundOrientedBox.Transform(box, GetWorldMatrix());
//	return box;
//}
//
//// Transformed Bounding Box
//BoundingBox IModelNode::GetBoundingBox() const {
//	BoundingBox box;
//	BoundBox.Transform(box, GetWorldMatrix());
//	return box;
//}
//
//// Transformed Bounding Sphere
//BoundingSphere IModelNode::GetBoundingSphere() const
//{
//	BoundingSphere sphere;
//	BoundSphere.Transform(sphere, GetWorldMatrix());
//	return sphere;
//}
//
//void XM_CALLCONV IModelNode::SetModelMatrix(DirectX::FXMMATRIX model)
//{
//	XMStoreFloat4x4(&LocalMatrix, model);
//}
//
//XMMATRIX IModelNode::GetModelMatrix() const
//{
//	return XMLoadFloat4x4(&LocalMatrix);
//}

//void XM_CALLCONV LocalMatrixHolder::SetModelMatrix(DirectX::FXMMATRIX model)
//{
//	XMStoreFloat4x4(&LocalMatrix,model);
//}
//
//XMMATRIX LocalMatrixHolder::GetModelMatrix() const
//{
//	return XMLoadFloat4x4(&LocalMatrix);
//}

void MonolithModel::Render(ID3D11DeviceContext * pContext, const Matrix4x4& transform, IEffect * pEffect)
{
	if (pEffect == nullptr && ModelPart::pEffect)
		pEffect = ModelPart::pEffect.get();

	auto pMat = dynamic_cast<IEffectMatrices*>(pEffect);
	if (pMat)
		pMat->SetWorld(transform);

	ModelPart::Render(pContext, pEffect);
}

std::shared_ptr<MeshBuffer> GeometricPrimtives::CreateCube()
{
	return std::shared_ptr<MeshBuffer>();
}

BoundingBox MonolithModel::GetBoundingBox() const
{
	return ModelPart::BoundBox;
}

BoundingOrientedBox MonolithModel::GetOrientedBoundingBox() const
{
	return ModelPart::BoundOrientedBox;
}

// Inherited via IModelNode

BoundingBox CollectionModel::GetBoundingBox() const { return BoundBox; }

//BoundingOrientedBox CollectionModel::GetOrientedBoundingBox() const { return BoundOrientedBox; }

// Inherited via IModelNode

BoundingBox CompositionModel::GetBoundingBox() const { return BoundBox; }

BoundingOrientedBox CompositionModel::GetOrientedBoundingBox() const { return BoundOrientedBox; }

IModelNode::~IModelNode()
{
}

// Check if the data have been loaded to CPU memery
bool DefaultSkinningModel::IsInMemery() const
{
	return m_Vertices != nullptr;
}

// Create GPU objects using Vertex/Index Buffer in CPU
bool DefaultSkinningModel::CreateDeviceResource(ID3D11Device *pDevice)
{
	auto pMajorBuffer = std::make_shared<MeshBuffer>();
	try
	{
		pMajorBuffer->CreateDeviceResources(pDevice, m_Vertices.get(), m_VertexCount, m_Indices.get(), m_IndexCount);

		for (auto& part : Parts)
		{
			part.pMesh->pVertexBuffer = pMajorBuffer->pVertexBuffer;
			part.pMesh->VertexStride = pMajorBuffer->VertexStride;
			part.pMesh->pIndexBuffer = pMajorBuffer->pIndexBuffer;
			part.pMesh->pInputLayout = pMajorBuffer->pInputLayout;
			part.pMesh->InputElementCount = pMajorBuffer->InputElementCount;
			part.pMesh->pInputElements = pMajorBuffer->pInputElements;
			part.pMesh->IndexFormat = pMajorBuffer->IndexFormat;
			part.pMesh->PrimitiveType = pMajorBuffer->PrimitiveType;

			part.pMaterial->CreateDeviceResources(pDevice, false);
		}
	}
	catch (const std::exception&)
	{
		return false;
	}
	return true;
}
// Release the Vertex/Index Buffer in CPU memery
void DefaultSkinningModel::ReleaseDynamicResource()
{
	m_Vertices.reset();
	m_Indices.reset();
	Vertices.reset();
	Facets.reset();
	Positions.reset();
	Normals.reset();
	TexCoords.reset();
	Tagents.reset();
	BlendWeights.reset();
	BlendIndices.reset();
}

void DefaultSkinningModel::ResetRanges()
{
	Vertices.reset(m_Vertices.get(), sizeof(VertexType), m_VertexCount);
	Facets.reset(reinterpret_cast<TriangleType*>(m_Indices.get()),
		sizeof(TriangleType),
		m_VertexCount / 3);
	Positions.reset(reinterpret_cast<Vector3*>(&m_Vertices[0].position),
		sizeof(VertexType), m_VertexCount);
	Normals.reset(reinterpret_cast<Vector3*>(&m_Vertices[0].normal),
		sizeof(VertexType), m_VertexCount);
	TexCoords.reset(reinterpret_cast<Vector2*>(&m_Vertices[0].textureCoordinate),
		sizeof(VertexType), m_VertexCount);
	Tagents.reset(reinterpret_cast<Vector4*>(&m_Vertices[0].tangent),
		sizeof(VertexType), m_VertexCount);
	BlendWeights.reset(&m_Vertices[0].weights,
		sizeof(VertexType), m_VertexCount);
	BlendIndices.reset(&m_Vertices[0].indices,
		sizeof(VertexType), m_VertexCount);
}

size_t DefaultSkinningModel::GetBonesCount() const
{
	return BoneTransforms.size();
}

DirectX::XMMATRIX * DefaultSkinningModel::GetBoneTransforms()
{
	return reinterpret_cast<XMMATRIX*>(BoneTransforms.data());
}

IDynamicAsset::~IDynamicAsset()
{

}

// Check if the GPU buffer is flaged as DYNAMIC

bool IDynamicAsset::IsDeviceResourceDynamic() const
{
	return false;
}

// Check if the data have been loaded to CPU memery

bool IDynamicAsset::IsInMemery() const { return false; }

bool IDynamicAsset::IsInDevice() const { return false; }

// Release the Vertex/Index Buffer in CPU memery


// Update GPU buffers using Vertex/Index Buffer in CPU

bool IDynamicAsset::UpdateDeviceResource(ID3D11DeviceContext * pContext)
{
	return false;
}

void IDynamicAsset::ReleaseDynamicResource()
{}

// To a amx binary stream , default to false

bool IDynamicAsset::Serialize(std::ostream & binary) const
{
	return false;
}

// from a amx binary stream

bool IDynamicAsset::Deserialize(std::istream & binary)
{
	return false;
}


DefaultSkinningModel::DefaultSkinningModel() {}

DefaultSkinningModel::~DefaultSkinningModel()
{

}

SkinMeshData::SkinMeshData()
{
	VertexCount = 0;
	IndexCount = 0;
	BonesCount = 0;
	Vertices = nullptr;
	Indices = nullptr;
}

void SkinMeshData::Release()
{
	if (Vertices)
	{
		delete Vertices;
		Vertices = nullptr;
	}
	if (Indices)
	{
		delete Indices;
		Indices = nullptr;
	}
}

void SkinMeshData::Serialize(std::ostream & binary) const
{
	binary << VertexCount << IndexCount << BonesCount << (uint32_t)sizeof(SkinMeshData::VertexType) << (uint32_t)sizeof(SkinMeshData::IndexType);
	binary.write(reinterpret_cast<const char*>(Vertices), sizeof(SkinMeshData::VertexType)*VertexCount);
	binary.write(reinterpret_cast<const char*>(Indices), sizeof(SkinMeshData::IndexType)*IndexCount);
}

void SkinMeshData::Deserialize(std::istream & binary)
{
	uint32_t VertexSizeInByte, IndexSizeInByte;
	binary >> VertexCount >> IndexCount >> BonesCount >> VertexSizeInByte >> IndexSizeInByte;

	assert(IndexCount % PolygonSize == 0);
	assert(sizeof(SkinMeshData::VertexType) == VertexSizeInByte);
	assert(sizeof(SkinMeshData::IndexType) == IndexSizeInByte);

	binary.read(reinterpret_cast<char*>(Vertices), sizeof(SkinMeshData::VertexType)*VertexCount);
	binary.read(reinterpret_cast<char*>(Indices), sizeof(SkinMeshData::IndexType)*IndexCount);
}


// To a amx binary stream
bool DefaultSkinningModel::Serialize(std::ostream& binary) const {
	SkinMeshData data;
	data.Serialize(binary);
	return true;
}
// from a amx binary stream
bool DefaultSkinningModel::Deserialize(std::istream& binary)
{
	SkinMeshData data;
	data.Deserialize(binary);
	return true;
}

// Reload from file to CPU
bool DefaultSkinningModel::Reload()
{
	return false;
}

void DefaultSkinningModel::Render(ID3D11DeviceContext * pContext, const Matrix4x4 & transform, IEffect * pEffect)
{
	if (pEffect == nullptr) pEffect = this->Parts[0].pEffect.get();
	auto pSkinning = dynamic_cast<IEffectSkinning*>(pEffect);
	if (pSkinning)
	{
		pSkinning->SetWeightsPerVertex(4U);
		//pSkinning->ResetBoneTransforms();
		pSkinning->SetBoneTransforms(reinterpret_cast<const XMMATRIX*>(BoneTransforms.data()), BoneTransforms.size());
	}
	else
	{
		//throw std::exception("Effect don't support skinning interface.");
	}
	CompositionModel::Render(pContext, transform, pEffect);
}


void DirectX::Scene::DefaultSkinningModel::SetFromSkinMeshData(std::list<SkinMeshData>& meshes, const std::wstring & textureDir)
{
	Parts.resize(meshes.size());
	// Merge Vertex Buffer
	int i = 0, tvSize = 0, tiSize = 0;

	for (auto& data : meshes)
	{
		tvSize += data.VertexCount;
		tiSize += data.IndexCount;
	}

	auto pVertices = meshes.front().Vertices;
	auto pIndices = meshes.front().Indices;

	if (meshes.size() > 1)
	{
		pVertices = new VertexType[tvSize];
		pIndices = new IndexType[tiSize];
		tvSize = 0, tiSize = 0;
		for (auto& data : meshes)
		{
			auto &part = Parts[i++];

			std::copy_n(data.Vertices, data.VertexCount, pVertices + tvSize);
			std::copy_n(data.Indices, data.IndexCount, pIndices + tiSize);

			part.Name = data.Name;
			part.pMesh = make_shared<MeshBuffer>();
			part.pMesh->VertexOffset = tvSize;
			part.pMesh->StartIndex = tiSize;
			part.pMesh->VertexCount = data.VertexCount;
			part.pMesh->IndexCount = data.IndexCount;

			part.pMaterial = PhongMaterial::CreateFromMaterialData(data.Material, textureDir);
			
			tvSize += data.VertexCount;
			tiSize += data.IndexCount;

			DirectX::CreateBoundingBoxesFromPoints(part.BoundBox, part.BoundOrientedBox,
				data.VertexCount, &data.Vertices[0].position, sizeof(SkinMeshData::VertexType));

			delete data.Vertices;
			data.Vertices = nullptr;
			delete data.Indices;
			data.Indices = nullptr;
		}
	}

	m_VertexCount = tvSize;
	m_IndexCount = tiSize;
	m_BonesCount = meshes.front().BonesCount;
	m_Vertices.reset(pVertices);
	m_Indices.reset(pIndices);

	BoneTransforms.resize(m_BonesCount);
	DirectX::CreateBoundingBoxesFromPoints(BoundBox, BoundOrientedBox,
		m_VertexCount, &m_Vertices[0].position, sizeof(SkinMeshData::VertexType));
	ResetRanges();
}

void DefaultSkinningModel::SetFromSkinMeshData(SkinMeshData* pData, const std::wstring& textureDir)
{
	m_VertexCount = pData->VertexCount;
	m_IndexCount = pData->IndexCount;
	m_BonesCount = pData->BonesCount;
	m_Vertices.reset(pData->Vertices);
	m_Indices.reset(pData->Indices);

	Parts.emplace_back();
	Parts[0].Name = pData->Name;
	Parts[0].pMesh = std::make_shared<MeshBuffer>();

	Parts[0].pMesh->VertexCount = m_VertexCount;
	Parts[0].pMesh->IndexCount = m_IndexCount;

	Parts[0].pMaterial = PhongMaterial::CreateFromMaterialData(pData->Material, textureDir);

	BoneTransforms.resize(m_BonesCount);
	DirectX::CreateBoundingBoxesFromPoints(BoundBox, BoundOrientedBox,
		m_VertexCount, &m_Vertices[0].position, sizeof(SkinMeshData::VertexType));
	ResetRanges();
}

DefaultSkinningModel* DefaultSkinningModel::CreateFromAmxFile(const std::string& file, ID3D11Device* pDevice)
{
	SkinMeshData data;
	std::ifstream fin(file, std::ios::binary);
	data.Deserialize(fin);
	auto pModel = new DefaultSkinningModel();
	pModel->SetFromSkinMeshData(&data);
	if (pDevice)
		pModel->CreateDeviceResource(pDevice);
	return pModel;
}

DefaultSkinningModel * DefaultSkinningModel::CreateFromData(SkinMeshData * pData, const std::wstring& textureDir, ID3D11Device* pDevice)
{
	auto pModel = new DefaultSkinningModel();
	pModel->SetFromSkinMeshData(pData, textureDir);
	if (pDevice)
	{
		pModel->CreateDeviceResource(pDevice);
	}
	return pModel;
}

DefaultSkinningModel * DirectX::Scene::DefaultSkinningModel::CreateFromDatas(std::list<SkinMeshData>& datas, const std::wstring& textureDir, ID3D11Device * pDevice)
{
	auto pModel = new DefaultSkinningModel();
	pModel->SetFromSkinMeshData(datas, textureDir);

	if (pDevice)
	{
		pModel->CreateDeviceResource(pDevice);
	}
	return pModel;
}

std::shared_ptr<MeshBuffer> MeshBuffer::CreateCube(ID3D11Device * pDevice, float size, bool rhcoords)
{
	typedef VertexPositionNormalTangentColorTextureSkinning VertexType;
	typedef uint16_t IndexType;
	// A cube has six faces, each one pointing in a different direction.
	const int FaceCount = 6;

	static const XMVECTORF32 faceNormals[FaceCount] =
	{
		{ 0,  0,  1 },
		{ 0,  0, -1 },
		{ 1,  0,  0 },
		{ -1,  0,  0 },
		{ 0,  1,  0 },
		{ 0, -1,  0 },
	};

	static const XMVECTORF32 textureCoordinates[4] =
	{
		{ 1, 0 },
		{ 1, 1 },
		{ 0, 1 },
		{ 0, 0 },
	};

	std::vector<VertexType> vertices;
	std::vector<IndexType> indices;

	size *= 0.5f;

	// Create each face in turn.
	for (int i = 0; i < FaceCount; i++)
	{
		XMVECTOR normal = faceNormals[i];

		// Get two vectors perpendicular both to the face normal and to each other.
		XMVECTOR basis = (i >= 4) ? g_XMIdentityR2 : g_XMIdentityR1;

		XMVECTOR side1 = XMVector3Cross(normal, basis);
		XMVECTOR side2 = XMVector3Cross(normal, side1);

		// Six indices (two triangles) per face.
		size_t vbase = vertices.size();
		indices.push_back(vbase + 0);
		indices.push_back(vbase + 1);
		indices.push_back(vbase + 2);

		indices.push_back(vbase + 0);
		indices.push_back(vbase + 2);
		indices.push_back(vbase + 3);

		XMVECTOR tagent = g_XMOne;
		XMVECTOR color = Colors::White;
		XMUINT4 bindices;
		XMVECTOR bweights = g_XMIdentityR0;
		// Four vertices per face.
		vertices.push_back(VertexType((normal - side1 - side2) * size, normal, tagent, color, textureCoordinates[0], bindices, bweights));
		vertices.push_back(VertexType((normal - side1 + side2) * size, normal, tagent, color, textureCoordinates[1], bindices, bweights));
		vertices.push_back(VertexType((normal + side1 + side2) * size, normal, tagent, color, textureCoordinates[2], bindices, bweights));
		vertices.push_back(VertexType((normal + side1 - side2) * size, normal, tagent, color, textureCoordinates[3], bindices, bweights));
	}

	assert((indices.size() % 3) == 0);

	if (rhcoords)
	{
		for (auto it = indices.begin(); it != indices.end(); it += 3)
		{
			std::swap(*it, *(it + 2));
		}
		for (auto it = vertices.begin(); it != vertices.end(); ++it)
		{
			it->textureCoordinate.x = (1.f - it->textureCoordinate.x);
		}
	}

	auto pMeshBuffer = std::make_shared<MeshBuffer>();
	pMeshBuffer->CreateDeviceResources(pDevice, vertices.data(), vertices.size(), indices.data(), indices.size());

	return pMeshBuffer;
}
