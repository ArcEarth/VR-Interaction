////////////////////
// GLOBALS BUFFERS//
////////////////////
// 
// This value is up to 1024 since each cbuffer can only hold 4096 vectors
// Shader model 4 can only support 256

#define MAX_BLEND_MATRIX_COUNT 255
#define VERTEX_BLEND_BONES 4

cbuffer ObjectBuffer
{
	matrix WorldMatrix;
	float4x3 BlendMatrices[MAX_BLEND_MATRIX_COUNT];
}

cbuffer CamareBuffer
{
	matrix ViewProjectionMatrix;
	float4 CameraPosition;
	float4 CameraFocus;
}


#include "Structures.hlsli"
#include "Common.hlsli"


VSOutputPixelLightingTx SkinningVS( VSInputNmTxWeights input )
{    
	VSOutputPixelLightingTx output;

	Skin(input,VERTEX_BLEND_BONES);

	output.PositionWS = input.Position;
	output.PositionPS = mul(input.Position,ViewProjectionMatrix);
	output.NormalWS = input.Normal;
	output.TexCoord = input.TexCoord;
//	output.Diffuse = float4(1.0f,1.0f,1.0f,1.0f);
	output.Diffuse = input.Position;

	return output;
}