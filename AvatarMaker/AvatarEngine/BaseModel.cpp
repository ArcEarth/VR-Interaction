#include "BaseModel.h"

using namespace std;


BaseModel::BaseModel(void)
{
	m_IsInitialized = false;
	m_pVertexShader = 0;
	m_pGeometryShader = 0;
	m_pPixelShader = 0;
	m_pLayout = 0;
	m_pVertexBuffer = m_pIndexBuffer = m_pConstantBuffer = 0;
	D3DXMatrixIdentity(&m_LocalTransformMatrix);
	m_VertexCount = m_IndexCount = 0;
}


BaseModel::~BaseModel(void)
{
	if (m_IsInitialized)
		Release();
}

HRESULT BaseModel::Initialize(ID3D11Device *pDevice,ID3D11VertexShader *pVertexShader,ID3D11GeometryShader *pGeometryShader,ID3D11PixelShader *pPixelShader,ID3D11Buffer *pConstantBuffer,ID3D11InputLayout* pLayout){
	m_pVertexShader=pVertexShader;
	m_pGeometryShader=pGeometryShader;
	m_pPixelShader=pPixelShader;
	m_pConstantBuffer=pConstantBuffer;
	m_pLayout=pLayout;

	return S_OK;
}

bool BaseModel::Load(ID3D11Device* pDevice, const vector<D3DXVECTOR3> &Vertices,const vector<TRIANGLE> &Triangles,D3DXVECTOR4 Color,const D3DXMATRIX *LocalTransformMatrix){
	VertexType* vertices;
	unsigned long* indices;
	D3D11_BUFFER_DESC vertexBufferDesc, indexBufferDesc;
    D3D11_SUBRESOURCE_DATA vertexData, indexData;
	HRESULT result;
	int i;

	m_VertexCount = Vertices.size();
	m_IndexCount = Triangles.size()*3;

	if (LocalTransformMatrix != NULL)
		m_LocalTransformMatrix = *LocalTransformMatrix;
	else
	{
		D3DXMatrixIdentity(&m_LocalTransformMatrix);
	}

	// Create the vertex array.
	vertices = new VertexType[m_VertexCount];
	if(!vertices)
	{
		return false;
	}

	// Create the index array.
	indices = new unsigned long[m_IndexCount];
	if(!indices)
	{
		return false;
	}

	// Load the vertex array and index array with data.
	for(i=0; i<m_VertexCount; i++)
	{
		vertices[i].position = Vertices[i];
		vertices[i].color = Color;
	}

	for(i=0; i<Triangles.size();i++)
	{
		indices[i*3+0]=Triangles[i].v0;
		indices[i*3+1]=Triangles[i].v1;
		indices[i*3+2]=Triangles[i].v2;
	}

	// Set up the description of the static vertex buffer.
    vertexBufferDesc.Usage = D3D11_USAGE_DEFAULT;
    vertexBufferDesc.ByteWidth = sizeof(VertexType) * m_VertexCount;
    vertexBufferDesc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
    vertexBufferDesc.CPUAccessFlags = 0;
    vertexBufferDesc.MiscFlags = 0;
	vertexBufferDesc.StructureByteStride = 0;

	// Give the subresource structure a pointer to the vertex data.
    vertexData.pSysMem = vertices;
	vertexData.SysMemPitch = 0;
	vertexData.SysMemSlicePitch = 0;

	// Now create the vertex buffer.
    result = pDevice->CreateBuffer(&vertexBufferDesc, &vertexData, &m_pVertexBuffer);
	if(FAILED(result))	return false;

	// Set up the description of the static index buffer.
    indexBufferDesc.Usage = D3D11_USAGE_DEFAULT;
    indexBufferDesc.ByteWidth = sizeof(unsigned long) * m_IndexCount;
    indexBufferDesc.BindFlags = D3D11_BIND_INDEX_BUFFER;
    indexBufferDesc.CPUAccessFlags = 0;
    indexBufferDesc.MiscFlags = 0;
	indexBufferDesc.StructureByteStride = 0;

	// Give the subresource structure a pointer to the index data.
    indexData.pSysMem = indices;
	indexData.SysMemPitch = 0;
	indexData.SysMemSlicePitch = 0;

	// Create the index buffer.
	result = pDevice->CreateBuffer(&indexBufferDesc, &indexData, &m_pIndexBuffer);
	if(FAILED(result))	return false;

	// Release the arrays now that the vertex and index buffers have been created and loaded.
	delete [] vertices;
	vertices = 0;

	delete [] indices;
	indices = 0;

	m_IsInitialized = true;
	m_ModelType = Model_Tiangles;
	return true;
}

bool BaseModel::Load(ID3D11Device* pDevice, const vector<D3DXVECTOR3> &Vertices,const vector<pair<int,int>> &Lines,D3DXVECTOR4 Color,const D3DXMATRIX *LocalTransformMatrix){
	VertexType* vertices;
	unsigned long* indices;
	D3D11_BUFFER_DESC vertexBufferDesc, indexBufferDesc;
    D3D11_SUBRESOURCE_DATA vertexData, indexData;
	HRESULT result;
	int i;

	m_VertexCount = Vertices.size();
	m_IndexCount = Lines.size()*2;

	if (LocalTransformMatrix != NULL)
		m_LocalTransformMatrix = *LocalTransformMatrix;
	else
	{
		D3DXMatrixIdentity(&m_LocalTransformMatrix);
	}

	// Create the vertex array.
	vertices = new VertexType[m_VertexCount];
	if(!vertices)
	{
		return false;
	}

	// Create the index array.
	indices = new unsigned long[m_IndexCount];
	if(!indices)
	{
		return false;
	}

	// Load the vertex array and index array with data.
	for(i=0; i<m_VertexCount; i++)
	{
		vertices[i].position = Vertices[i];
		vertices[i].color = Color;
	}

	for(i=0; i<Lines.size();i++)
	{
		indices[i*2+0]=Lines[i].first;
		indices[i*2+1]=Lines[i].second;
	}

	// Set up the description of the static vertex buffer.
    vertexBufferDesc.Usage = D3D11_USAGE_DEFAULT;
    vertexBufferDesc.ByteWidth = sizeof(VertexType) * m_VertexCount;
    vertexBufferDesc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
    vertexBufferDesc.CPUAccessFlags = 0;
    vertexBufferDesc.MiscFlags = 0;
	vertexBufferDesc.StructureByteStride = 0;

	// Give the subresource structure a pointer to the vertex data.
    vertexData.pSysMem = vertices;
	vertexData.SysMemPitch = 0;
	vertexData.SysMemSlicePitch = 0;

	// Now create the vertex buffer.
    result = pDevice->CreateBuffer(&vertexBufferDesc, &vertexData, &m_pVertexBuffer);
	if(FAILED(result))	return false;

	// Set up the description of the static index buffer.
    indexBufferDesc.Usage = D3D11_USAGE_DEFAULT;
    indexBufferDesc.ByteWidth = sizeof(unsigned long) * m_IndexCount;
    indexBufferDesc.BindFlags = D3D11_BIND_INDEX_BUFFER;
    indexBufferDesc.CPUAccessFlags = 0;
    indexBufferDesc.MiscFlags = 0;
	indexBufferDesc.StructureByteStride = 0;

	// Give the subresource structure a pointer to the index data.
    indexData.pSysMem = indices;
	indexData.SysMemPitch = 0;
	indexData.SysMemSlicePitch = 0;

	// Create the index buffer.
	result = pDevice->CreateBuffer(&indexBufferDesc, &indexData, &m_pIndexBuffer);
	if(FAILED(result))	return false;

	// Release the arrays now that the vertex and index buffers have been created and loaded.
	delete [] vertices;
	vertices = 0;

	delete [] indices;
	indices = 0;

	m_IsInitialized = true;
	m_ModelType = Model_Lines;
	return true;
}

bool BaseModel::Reload(ID3D11Device* pDevice, const vector<VERTEX> &Vertices,const vector<TRIANGLE> &Triangles,D3DXVECTOR4 Color,const D3DXMATRIX *LocalTransformMatrix)
{
	bool hr;
	ReleaseBuffers();
	hr = Load(pDevice,Vertices,Triangles,Color,LocalTransformMatrix);
	return hr;
}

bool BaseModel::Reload(ID3D11Device* pDevice, const vector<VERTEX> &Vertices,const vector<pair<int,int>> &Lines,D3DXVECTOR4 Color,const D3DXMATRIX *LocalTransformMatrix)
{
	bool hr;
	ReleaseBuffers();
	hr = Load(pDevice,Vertices,Lines,Color,LocalTransformMatrix);
	return hr;
}

void BaseModel::SetLocalTransformMatrix(const D3DXMATRIX &LocalTransformMatrix)
{
	m_LocalTransformMatrix = LocalTransformMatrix;
}

void BaseModel::ReleaseBuffers(){
	// Release the index buffer.
	if(m_pIndexBuffer)
	{
		m_pIndexBuffer->Release();
		m_pIndexBuffer = 0;
	}

	// Release the vertex buffer.
	if(m_pVertexBuffer)
	{
		m_pVertexBuffer->Release();
		m_pVertexBuffer = 0;
	}
}

void BaseModel::Release(){
	ReleaseBuffers();
	m_IsInitialized = false;
	m_ModelType = Model_Null;
}

void BaseModel::Render(ID3D11DeviceContext *pContext,const D3DXMATRIX *pWorldMatrix,const D3DXMATRIX *pViewMatrix, D3DXMATRIX *pProjectionMatrix){
	unsigned int stride;
	unsigned int offset;
	// Set vertex buffer stride and offset.
	stride = sizeof(VertexType); 
	offset = 0;

	//// Set the input assemabler stage
	pContext->IASetInputLayout(m_pLayout);

	pContext->IASetVertexBuffers(0, 1, &m_pVertexBuffer, &stride, &offset);
	pContext->IASetIndexBuffer(m_pIndexBuffer, DXGI_FORMAT_R32_UINT, 0);
	switch (m_ModelType)
	{
	case BaseModel::Model_Tiangles:
		pContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
		break;
	case BaseModel::Model_Lines:
		pContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST);
		break;
	}

	ShaderConstants *Constant;

    D3D11_MAPPED_SUBRESOURCE mappedResource;

	//We can't use this since we define this ConstantBuffer as CPU_ACESS_WIRTE , this only works we the buffer are cpu-acess-denied
	//	pContext->UpdateSubresource(m_pConstantBuffer, 0, NULL, &Constant, 0, 0);

	pContext->Map(m_pConstantBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &mappedResource);
	Constant = (ShaderConstants*)mappedResource.pData;

	//Transform with the local matrix and transpose then
	D3DXMatrixMultiplyTranspose(&Constant->world,&m_LocalTransformMatrix,pWorldMatrix);
	D3DXMatrixTranspose(&Constant->view, pViewMatrix);
	D3DXMatrixTranspose(&Constant->projection, pProjectionMatrix);
    pContext->Unmap(m_pConstantBuffer, 0);

	pContext->VSSetConstantBuffers(0, 1 , &m_pConstantBuffer);
	pContext->VSSetShader(m_pVertexShader,NULL,0);
	pContext->GSSetShader(m_pGeometryShader,NULL,0);
	pContext->PSSetShader(m_pPixelShader,NULL,0);

	pContext->DrawIndexed(m_IndexCount, 0, 0);
	return;
}

BaseModelFactory::BaseModelFactory()
{
	m_ColorShader = 0;
}

BaseModelFactory::~BaseModelFactory()
{
	Release();
}

HRESULT BaseModelFactory::Initialize(ID3D11Device *pDevice){
	bool hr;
	m_ColorShader = new ColorShaderClass;
	hr = m_ColorShader->Initialize(pDevice,NULL);
	if (hr) 
		return S_OK;
	else
		return S_FALSE;
}

//Release the factor will release the shader this model class use , careful!
void BaseModelFactory::Release(){
	if (m_ColorShader) {
		m_ColorShader->Shutdown();
		m_ColorShader = 0;
	}
}

BaseModel* BaseModelFactory::CreateBaseModel(ID3D11Device *pDevice){
	BaseModel *pBaseModel;
	pBaseModel = new BaseModel;
	pBaseModel->Initialize(pDevice,m_ColorShader->m_vertexShader,NULL,m_ColorShader->m_pixelShader,m_ColorShader->m_matrixBuffer,m_ColorShader->m_layout);
	return pBaseModel;
}

BaseModel* BaseModelFactory::CreateRectangleBoard(ID3D11Device *pDevice,const D3DXVECTOR3 &Position,float Width , float Height , D3DXVECTOR4 Color){
	BaseModel *pModel;

	pModel = CreateBaseModel(pDevice);
	
	ReloadRectangleBoard(pModel,pDevice,Position,Width,Height,Color);

	return pModel;
}

bool BaseModelFactory::ReloadRectangleBoard(BaseModel* pModel, ID3D11Device *pDevice, const D3DXVECTOR3 &Position, float Width , float Height , D3DXVECTOR4 Color){
	vector<VERTEX> Vertices;
	Vertices.push_back(D3DXVECTOR3(	-Width/2,	Height/2,	0.0f));
	Vertices.push_back(D3DXVECTOR3(	Width/2,	Height/2,	0.0f));
	Vertices.push_back(D3DXVECTOR3(	-Width/2,	-Height/2,	0.0f));
	Vertices.push_back(D3DXVECTOR3(	Width/2,	-Height/2,	0.0f));
	//Vertices.push_back(D3DXVECTOR3(	-1.0f,	1.0f,	1.0f));
	//Vertices.push_back(D3DXVECTOR3(	1.0f,	1.0f,	1.0f));
	//Vertices.push_back(D3DXVECTOR3(	-1.0f,	-1.0f,	1.0f));

	vector<TRIANGLE> Triangles;
	Triangles.push_back(TRIANGLE(0,1,2));
	Triangles.push_back(TRIANGLE(3,2,1));

	//Triangles.push_back(TRIANGLE(2,1,0));
	//Triangles.push_back(TRIANGLE(1,2,3));

	D3DXMATRIX TransformMatrix;
	D3DXMatrixTranslation(&TransformMatrix,Position.x,Position.y,Position.z);

	bool hr = pModel->Reload(pDevice,Vertices,Triangles,Color,&TransformMatrix);
	return hr;
}