#include "Model.h"

using namespace DirectX::Scene;
using namespace DirectX;
using namespace std;

template <class T>
class IEnumerable
{
};

template <class T>
struct stride_vector
{
public:
	//std::vector<U>& source;
	T*	   data;
	size_t stride = sizeof(T);
	size_t size;
	T& operator[](size_t idx)
	{
#if _ITERATOR_DEBUG_LEVEL == 2
		if (size <= idx)
		{	// report error
			_DEBUG_ERROR("vector subscript out of range");
			_SCL_SECURE_OUT_OF_RANGE;
		}

#elif _ITERATOR_DEBUG_LEVEL == 1
		_SCL_SECURE_VALIDATE_RANGE(idx < size);
#endif /* _ITERATOR_DEBUG_LEVEL */

		return *reinterpret_cast<T*>(reinterpret_cast<byte*>(data) + stride);
	}
	const T& operator[](size_t idx)
	{
#if _ITERATOR_DEBUG_LEVEL == 2
		if (size <= idx)
		{	// report error
			_DEBUG_ERROR("vector subscript out of range");
			_SCL_SECURE_OUT_OF_RANGE;
		}

#elif _ITERATOR_DEBUG_LEVEL == 1
		_SCL_SECURE_VALIDATE_RANGE(idx < size);
#endif /* _ITERATOR_DEBUG_LEVEL */

		return *reinterpret_cast<const T*>(reinterpret_cast<const byte*>(data) + stride);
	}
	//void push_back(const T& entity)
	//{
	//	if (size < source.size())
	//		(*this)[size++] = entity;
	//	else
	//	{
	//		source.push_back
	//	}
	//}
};


class BasicVertexCollection : public std::vector<DirectX::VertexPositionNormalTexture>
{
public:


	using std::vector<DirectX::VertexPositionNormalTexture>::operator[];

	stride_vector<Vector3> Positions;
	stride_vector<Vector3> Normals;
	stride_vector<Vector3> Tagents;
	stride_vector<Vector2> TextureCoordinates;
	stride_vector<Color>   Colors;
	stride_vector<Vector4> BlendWeights;
};

void DirectX::Scene::Mesh::Render(ID3D11DeviceContext *pContext)
{
	if (pInputLayout)
		pContext->IASetInputLayout(pInputLayout.Get());

	// Set the input assembler stage
	auto vb = pVertexBuffer.Get();
	UINT vbStride = VertexStride;
	UINT vbOffset = 0;

	pContext->IASetVertexBuffers(0, 1, &vb, &vbStride, &vbOffset);


	if (pEffect)
		pEffect->Apply(pContext);

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

std::shared_ptr<ObjMesh> DirectX::Scene::ObjMesh::CreateFromFile(std::wstring file)
{
	std::ifstream stream;
	stream.open(file);
	string tag;

	Vector3 vec3;
	Vector2 vec2;
	Color color;
	XMUINT3 tri;

	color.w = 1.0f;
	while (!stream.eof())
	{
		stream >> tag;

		if (tag == "v")
		{
			stream >> vec3.x >> vec3.y >> vec3.z;
			m_Vertices.Postions.push_back(vec3);
		}
		else if (tag == "vc") // This is not inside the standard
		{
			stream >> color.x >> color.y >> color.z;
			m_Vertices.Colors.emplace_back(color);
		}
		else if (tag == "vt")
		{
			stream >> vec2.x >> vec2.y;
			m_Vertices.TexCoords.push_back(vec2);
		}
		else if (tag == "vn")
		{
			stream >> vec3.x >> vec3.y >> vec3.z;
			m_Vertices.Normals.push_back(vec3);
		}
		else if (tag == "vp")
		{
			stream >> vec3.x >> vec3.y >> vec3.z; //Ignore the unknow parameter data
												  //VertexParameters.push_back(vec3);
		}
		else if (tag == "f")
		{
			stream >> tri.x >> tri.y >> tri.z;
			tri.x--, tri.y--, tri.z--; // The obj file format starts the indices from 1, but as usual, it should starts from 0
			m_Facets.push_back(tri);
		}
	}
	auto N = m_Vertices.Count();
	assert(m_Vertices.TexCoords.size() == 0 || N == m_Vertices.TexCoords.size());
	//assert(m_Vertices.Normals.size() == 0 || N == m_Vertices.Normals.size());

	BoundingBox::CreateFromPoints(m_BoundingBox, m_Vertices.Postions.size(), &m_Vertices.Postions[0], sizeof(DirectX::SimpleMath::Vector3));

	if (!HasVertexNormal())
		GenerateVertexNormal();
	return std::shared_ptr<Mesh>();
}
