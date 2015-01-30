#ifndef BASE_MODEL_H
#define BASE_MODEL_H
#pragma once

#include <d3d11_1.h>
#include <D3Dcompiler.h>
#include <D3DX10math.h>
#include <vector>
#include "Polygonizer.h"
#include "colorshaderclass.h"

#pragma comment (lib, "D3DCompiler.lib")

using namespace std;

class BaseModel
{
protected:
	struct ShaderConstants {
		D3DXMATRIX world;
		D3DXMATRIX view;
		D3DXMATRIX projection;
	};
	struct VertexType
	{
		D3DXVECTOR3 position;
		D3DXVECTOR4 color;
	};
public:
	enum ModelType {
		Model_Null,
		Model_Tiangles,
		Model_Lines,
	};
public:
	BaseModel(void);
	~BaseModel(void);

	virtual HRESULT Initialize(ID3D11Device *pDevice,ID3D11VertexShader *pVertexShader,ID3D11GeometryShader *pGeometryShader,ID3D11PixelShader *pPixelShader,ID3D11Buffer *pConstantBuffer,ID3D11InputLayout* pLayout);

	virtual void Release();
	virtual void Render(ID3D11DeviceContext *pContext,const D3DXMATRIX *pWorldMatrix,const D3DXMATRIX *pViewMatrix, D3DXMATRIX *pProjectionMatrix);

	bool Load	(ID3D11Device* pDevice, const vector<VERTEX> &Vertices,const vector<TRIANGLE> &Triangles,D3DXVECTOR4 Color,const D3DXMATRIX *LocalTransformMatrix);
	bool Load	(ID3D11Device* pDevice, const vector<VERTEX> &Vertices,const vector<pair<int,int>> &Lines,D3DXVECTOR4 Color,const D3DXMATRIX *LocalTransformMatrix);
	bool Reload	(ID3D11Device* pDevice, const vector<VERTEX> &Vertices,const vector<TRIANGLE> &Triangles,D3DXVECTOR4 Color,const D3DXMATRIX *LocalTransformMatrix);
	bool Reload	(ID3D11Device* pDevice, const vector<VERTEX> &Vertices,const vector<pair<int,int>> &Lines,D3DXVECTOR4 Color,const D3DXMATRIX *LocalTransformMatrix);

	void SetLocalTransformMatrix(const D3DXMATRIX &LocalTransformMatrix);

protected:
	void ReleaseBuffers();

protected:
	//Shaders
	ID3D11VertexShader *m_pVertexShader;
	ID3D11GeometryShader *m_pGeometryShader;
	ID3D11PixelShader *m_pPixelShader;
	ID3D11InputLayout *m_pLayout;
	//Buffers for shader to render
	ID3D11Buffer *m_pVertexBuffer, *m_pIndexBuffer, *m_pConstantBuffer;

	//Local transform matrix represent position and orientation of this object
	D3DXMATRIX m_LocalTransformMatrix;
public:
	int m_VertexCount , m_IndexCount;
	bool m_IsInitialized;
	ModelType m_ModelType;
//	friend BaseModelFactory;
};

class BaseModelFactory
{
public:
	BaseModelFactory();
	~BaseModelFactory();

	HRESULT Initialize(ID3D11Device *pDevice);
	void Release();

	BaseModel* CreateBaseModel(ID3D11Device *pDevice);
	BaseModel* CreateRectangleBoard(ID3D11Device *pDevice,const D3DXVECTOR3 &Position, float Width , float Height , D3DXVECTOR4 Color);

	static bool ReloadRectangleBoard(BaseModel* pModel, ID3D11Device *pDevice, const D3DXVECTOR3 &Position, float Width , float Height , D3DXVECTOR4 Color);

private:
	ColorShaderClass *m_ColorShader;
};
#endif