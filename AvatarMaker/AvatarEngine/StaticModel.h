#pragma once
#include "DXBaseModel.h"
#include <DirectXMath.h>
#include "VertexTypes.h"
#include "DDSTextureLoader.h"
#include <fstream>

struct StaticObjectBuffer
{
	DirectX::XMFLOAT4X4 ModelMatrix;
};

typedef DirectX::BaseModel<DirectX::VertexPositionNormalTexture,StaticObjectBuffer>
	StaticModelBase;


template <typename VertexType,typename ConstantType> 
class DXTextureModel : public DirectX::BaseModel<VertexType,ConstantType>
{
public:
	typedef DirectX::BaseModel<VertexType,ConstantType> base_type;

	DXTextureModel(ID3D11Device *pDevice , const char * ModelFileName , const char * DDSTextureFileName)
		: base_type(pDevice)
	{
		// Create the texture
		HRESULT hr = DirectX::CreateDDSTextureFromFile(pDevice,DDSTextureFileName,nullptr,&m_pTextureView);
		DirectX::ThrowIfFailed(hr);
	}
	DXTextureModel(ID3D11Device *pDevice , ID3D11ShaderResourceView * pTextureView)
		: base_type(pDevice)
	{
		m_pTextureView = pTextureView;
	}
private:
	void LoadModelFromFile(const char* FileName){
		std::ifstream fin(FileName);
		char input;
		int vertexCount;
		// If it could not open the file then exit.
		if(fin.fail())
			throw new std::exception("Can't find model file");

		// Read up to the value of vertex count.
		fin.get(input);
		while(input != ':') fin.get(input);
		// Read in the vertex count.
		fin >> vertexCount;
		auto &Vertices = Vertices_W();
		Vertices.resize(vertexCount);
		Indices_W().resize(vertexCount);
		for (int i = 0; i < vertexCount; i++)
			Indices_W()[i] = i;
		fin.get(input);
		while(input != ':') fin.get(input);
		// Read in the vertex data.
		for(int i=0; i<vertexCount; i++)
		{
			fin >> Vertices[i].position.x >> Vertices[i].position.y >> Vertices[i].position.z;
			fin >> Vertices[i].textureCoordinate.x >> Vertices[i].textureCoordinate.y;
			fin >> Vertices[i].normal.x >> Vertices[i].normal.y >> Vertices[i].normal.z;
		}
		fin.close();
	}
	ID3D11ShaderResourceView *m_pTextureView;
};