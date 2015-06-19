#include "pch_bcl.h"
#include "SkinningModel.h"
#include "FbxParser.h"
#include <Effects.h>
using namespace Causality;

SkinAnimateModel::Ref Causality::SkinAnimateModel::CreateFromSkinMeshData(ID3D11Device * pDevice, SkinMeshData * pData)
{
	auto pModel = new SkinAnimateModel();
	auto & pMesh = pModel->pMesh = std::make_shared<DirectX::Scene::MeshBuffer>();
	pMesh->CreateDeviceResources(pDevice, pData->Vertices, pData->VertexCount, pData->Indices, pData->IndexCount);
	DirectX::BoundingBox::CreateFromPoints(pModel->BoundBox, pData->VertexCount, &pData->Vertices[0].position, sizeof(SkinMeshData::VertexType));
	DirectX::CreateBoundingOrientedBoxFromPoints(pModel->BoundOrientedBox, pData->VertexCount, &pData->Vertices[0].position, sizeof(SkinMeshData::VertexType));
	auto pEffect = std::make_shared<DirectX::SkinnedEffect>(pDevice);
	pModel->pEffect = pEffect;
	pEffect->SetWeightsPerVertex(4U);
	return pModel;
}

SkinAnimateModel::SkinAnimateModel()
{
}


SkinAnimateModel::~SkinAnimateModel()
{
}
