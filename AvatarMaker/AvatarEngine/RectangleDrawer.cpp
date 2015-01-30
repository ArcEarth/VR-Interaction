#include "stdafx.h"
#include "RectangleDrawer.h"
#include <D3Dcompiler.h>

#pragma comment (lib, "D3DCompiler.lib")

// RectDrawer helper class

RectDrawer::RectDrawer() :
	m_pBuffer(NULL),
	m_pPixelShader(NULL),
	m_pVertexShader(NULL),
	m_inited(false)
{
}
		
RectDrawer::~RectDrawer() {
	if(m_pBuffer) m_pBuffer->Release();
	if(m_pPixelShader) m_pPixelShader->Release();
	if(m_pVertexShader) m_pVertexShader->Release();
}
		
bool RectDrawer::Initialize(ID3D11Device *pDevice) {
	const char vsQuad[] =
	"cbuffer ShaderConstants : register(b0) {"
	"	float4x4 TransformMatrix : packoffset(c0);"
	"};"
	"float4 VS(uint VertexIndex : SV_VertexID) : SV_Position{"
	"	const float2 corners[4] = {"
	"		float2(-1.0f, 1.0f),"
	"		float2( 1.0f, 1.0f),"
	"		float2(-1.0f,-1.0f),"
	"		float2( 1.0f,-1.0f)"
	"	};"
	"	return mul(TransformMatrix, float4(corners[VertexIndex].xy, 0.0f, 1.0f));"
	"}";
	ID3DBlob *pCode;//
	HRESULT hResult = D3DCompile(vsQuad, sizeof(vsQuad), NULL, NULL, NULL, "VS", "vs_4_0", 0, 0, &pCode, NULL);
	if(FAILED(hResult)) return false;
	hResult = pDevice->CreateVertexShader(pCode->GetBufferPointer(), pCode->GetBufferSize(), NULL, &m_pVertexShader);
	pCode->Release();
	if(FAILED(hResult)) return false;
	
	const char psColor[] =
	"cbuffer ShaderConstants : register(b0) {"
	"	float4 Color : packoffset(c4);"
	"};"
	"float4 PS() : SV_Target {"
	"	return Color;"
	"}";
	hResult = D3DCompile(psColor, sizeof(psColor), NULL, NULL, NULL, "PS", "ps_4_0", 0, 0, &pCode, NULL);
	if(FAILED(hResult)) return false;
	hResult = pDevice->CreatePixelShader(pCode->GetBufferPointer(), pCode->GetBufferSize(), NULL, &m_pPixelShader);
	pCode->Release();
	if(FAILED(hResult)) return false;
	
	D3D11_BUFFER_DESC bufferDesc;
	ZeroMemory(&bufferDesc, sizeof(bufferDesc));
	bufferDesc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
	bufferDesc.ByteWidth = sizeof(ShaderConstants);
	bufferDesc.Usage = D3D11_USAGE_DEFAULT;
	hResult = pDevice->CreateBuffer(&bufferDesc, NULL, &m_pBuffer);
	if(FAILED(hResult)) return false;
	
	m_inited = true;
	
	return true;
}
		
void RectDrawer::DrawRect_Pixel(ID3D11DeviceContext *pContext, float left, float top, float right, float bottom , DirectX::FXMVECTOR Color) {
	if(!m_inited) return;
	
	ShaderConstants constants;
	ZeroMemory(&constants, sizeof(constants));
	D3D11_VIEWPORT vp; UINT nvp = 1;
	pContext->RSGetViewports(&nvp, &vp);

	float *m = (float*)constants.TransformMatrix.m;
	m[0] = (right - left) * 2.0f / vp.Width;
	m[12] = -1.0f + left * 2.0f / vp.Width;
	m[5] = (bottom - top) * -2.0f / vp.Height;
	m[13] = 1.0f + top * -2.0f / vp.Height;
	m[10] = 1.0f;
	m[15] = 1.0f;
	
	DirectX::XMStoreFloat4(&constants.Color,Color);

	Draw(pContext,constants);
}

void RectDrawer::DrawRect_ProjectionCoor(ID3D11DeviceContext *pContext, float left, float top, float right, float bottom , DirectX::FXMVECTOR Color) {
	if(!m_inited) return;
	
	ShaderConstants constants;
	ZeroMemory(&constants, sizeof(constants));
	
	float CX = 0.5f*(left + right);
	float CY = 0.5f*(top + bottom);
	float SX = 0.5f*std::abs(right - left);
	float SY = 0.5f*std::abs(top - bottom);

	constants.TransformMatrix.m[0][0] = SX;
	constants.TransformMatrix.m[1][1] = -SY;
	constants.TransformMatrix.m[2][2] = 1.0f;
	constants.TransformMatrix.m[3][3] = 1.0f;
	constants.TransformMatrix.m[3][0] = CX;
	constants.TransformMatrix.m[3][1] = CY;

	DirectX::XMStoreFloat4(&constants.Color,Color);

	Draw(pContext,constants);
}

void RectDrawer::DrawRect_ProjectionCoor_CenterExtent(ID3D11DeviceContext *pContext, float Center_X, float Center_Y, float Extent_X, float Extent_Y , DirectX::FXMVECTOR Color)
{
	ShaderConstants constants;
	ZeroMemory(&constants, sizeof(constants));
	
	constants.TransformMatrix.m[0][0] = Extent_X;
	constants.TransformMatrix.m[1][1] = Extent_Y;
	constants.TransformMatrix.m[2][2] = 1.0f;
	constants.TransformMatrix.m[3][3] = 1.0f;
	constants.TransformMatrix.m[3][0] = Center_X;
	constants.TransformMatrix.m[3][1] = Center_Y;

	DirectX::XMStoreFloat4(&constants.Color,Color);

	Draw(pContext,constants);
}

void RectDrawer::Draw(ID3D11DeviceContext *pContext , const ShaderConstants& Constants)
{
	if(!m_inited) return;
	pContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP);
	pContext->PSSetShader(m_pPixelShader, NULL, 0);
	pContext->PSSetConstantBuffers(0, 1, &m_pBuffer);
	pContext->VSSetShader(m_pVertexShader, NULL, 0);
	pContext->VSSetConstantBuffers(0, 1, &m_pBuffer);
	pContext->GSSetShader(NULL, NULL, 0);
	pContext->UpdateSubresource(m_pBuffer, 0, NULL, &Constants, 0, 0);
	pContext->Draw(4, 0);
}

