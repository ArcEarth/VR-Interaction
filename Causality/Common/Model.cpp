#include "Model.h"
#include <string>
#include "stride_iterator.h"
#include <sstream>
#include <WICTextureLoader.h>
#include <boost\filesystem.hpp>

using namespace DirectX::Scene;
using namespace DirectX;
using namespace std;

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

DirectX::Scene::ObjMesh::ObjMesh()
{
}

DirectX::Scene::ObjMesh::ObjMesh(const std::wstring &file)
{
	std::ifstream stream;
	stream.open(file);

	if (!stream.is_open())
		throw;
	string tag;
	Vector3 vec3;
	Vector2 vec2;
	Color color;
	FacetPrimitives::Triangle<IndexType> tri;

	std::vector<Vector3> positions;
	std::vector<Vector3> normals;
	std::vector<Vector2> texcoords;

	color.w = 1.0f;
	while (!stream.eof())
	{
		stream >> tag;

		if (tag == "v") // (x,y,z[,w]) coordinates, w is optional and defaults to 1.0.
		{
			stream >> vec3.x >> vec3.y >> vec3.z;
			positions.push_back(vec3);
		}
		else if (tag == "vt") // in (u, v [,w]), these will vary between 0 and 1, w is optional and defaults to 0.
		{
			stream >> vec2.x >> vec2.y;
			texcoords.push_back(vec2);
		}
		else if (tag == "vn") // (u, v [,w]) normals might not be unit. 
		{
			stream >> vec3.x >> vec3.y >> vec3.z;
			normals.push_back(vec3);
		}
		else if (tag == "vp") // ( u [,v] [,w] ) free form geometry statement
		{
			stream >> vec3.x >> vec3.y >> vec3.z; //Ignore the unknow parameter data
												  //VertexParameters.push_back(vec3);
		}
		else if (tag == "f")
		{
			stream >> tri.V0 >> tri.V1 >> tri.V2; // f v1 v2 v3 ...
												  // f v1/vt1/vn1 v2/vt2/vn2 v3/vt3/vn3 ...
												  // f v1//vn1 v2//vn2 v3//vn3 ...
			tri.V0--, tri.V1--, tri.V2--; // The obj file format starts the indices from 1, but as usual, it should starts from 0
			Facets.push_back(tri);
		}
	}

	auto N = positions.size();
	assert(normals.size() == 0 || N == normals.size());
	assert(texcoords.size() == 0 || N == texcoords.size());

	BoundingBox box;
	BoundingBox::CreateFromPoints(box, positions.size(), &positions[0], sizeof(Vector3));
	BoundingOrientedBox::CreateFromBoundingBox(Bound, box);
	Vertices.resize(N);

	Positions = stride_range<Vector3>((Vector3*) &Vertices[0].position, sizeof(VertexType), Vertices.size());
	std::copy_n(positions.begin(), N, Positions.begin());

	Normals = stride_range<Vector3>((Vector3*) &Vertices[0].normal, sizeof(VertexType), Vertices.size());
	if (normals.size() == 0)
	{
		GenerateNormal();
	}
	else
	{
		std::copy_n(normals.begin(), N, Normals.begin());
	}

	if (texcoords.size() > 0)
	{
		TexCoords = stride_range<Vector2>((Vector2*) &Vertices[0].textureCoordinate, sizeof(VertexType), Vertices.size());
		std::copy_n(texcoords.begin(), N, TexCoords.begin());
	}
}

inline void DirectX::Scene::ObjMesh::GenerateNormal()
{
	auto N = Vertices.size();
	//std::vector<Vector3> facetNormals;
	XMVECTOR n, v0, v1, v2;
	std::fill_n(Normals.begin(), N, Vector3());
	for (const auto& face : Facets)
	{
		v0 = Positions[face.V0];
		v1 = Positions[face.V1];
		v2 = Positions[face.V2];
		v1 -= v0;
		v2 -= v0;
		n = XMVector3Cross(v1, v2);
		n = XMVector3Normalize(n);
		if (XMVectorGetY(n) < 0.0f)
			n = XMVectorNegate(n);
		//facetNormals.emplace_back(n);
		Normals[face.V0] += n;
		Normals[face.V1] += n;
		Normals[face.V2] += n;
	}

	for (auto& nor : Normals)
	{
		nor.Normalize();
	}
}

DirectX::Scene::ObjMaterial::ObjMaterial()
{}

void DirectX::Scene::ObjMaterial::LoadFromFile(ID3D11Device* pDevice, const std::wstring &file)
{
	std::ifstream stream;
	stream.open(file);

	if (!stream.is_open())
		throw;

	string tag;
	string line;
	ComPtr<ID3D11DeviceContext> pContext;
	ComPtr<ID3D11Resource> pResource;

	pDevice->GetImmediateContext(&pContext);

	while (!stream.eof())
	{
		std::getline(stream, line);
		stringstream lss(line);
		lss >> tag;
		if (tag == "Ka")
		{ 
			lss >> AmbientColor.x >> AmbientColor.y >> AmbientColor.z;// >> AmbientColor.w;
		}
		else if (tag == "Kd")
		{
			lss >> DiffuseColor.x >> DiffuseColor.y >> DiffuseColor.z;// >> DiffuseColor.w;
		}
		else if (tag == "Ks")
		{
			lss >> SpecularColor.x >> SpecularColor.y >> SpecularColor.z;// >> DiffuseColor.w;
		}
		else if (tag == "d" || tag == "Tr")
		{
			lss >> Alpha;
		}
		else if (tag == "illum")
		{

		}
		else if (tag == "map_Ka")
		{

		}
		else if (tag == "map_Kd")
		{
			lss >> tag;
			wstring fileName(tag.begin(), tag.end());
			ThrowIfFailed(
				CreateWICTextureFromFile(pDevice, pContext.Get(), fileName.data(), &pResource, &ColorMap));
		}
		else if (tag == "map_Ks")
		{

		}
		else if (tag == "map_Ns")
		{

		}
		else if (tag == "map_d")
		{

		}
		else if (tag == "map_bump" || tag == "bump")
		{
			lss >> tag;
			wstring fileName(tag.begin(), tag.end());
			ThrowIfFailed(
				CreateWICTextureFromFile(pDevice, pContext.Get(), fileName.data(), &pResource, &BumpMap));
		}
		else if (tag == "map_disp" || tag == "disp")
		{
			lss >> tag;
			wstring fileName(tag.begin(), tag.end());
			ThrowIfFailed(
				CreateWICTextureFromFile(pDevice, pContext.Get(), fileName.data(), &pResource, &DisplaceMap));
		}
		else if (tag == "map_decal" || tag == "decal")
		{

		}
	}
}

using namespace std::tr2;
DirectX::Scene::ObjModel::ObjModel(ID3D11Device * pDevice, const std::wstring & file, const std::shared_ptr<IEffect>& pEffect)
	: ObjMesh(file)
{
	auto path = boost::filesystem::path(file);
	path.replace_extension(".mtl");
	ObjMaterial::LoadFromFile(pDevice,path.wstring());
	ObjMesh::Update(pDevice);
}
