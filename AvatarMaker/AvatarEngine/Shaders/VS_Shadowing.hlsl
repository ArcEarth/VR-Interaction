#include "Structures.hlsli"
#define VERTEX_BLEND_BONES 4

cbuffer CamareBuffer
{
	matrix ViewProjectionMatrix;
	float4 CameraPosition;
	float4 CameraFocus;
}

cbuffer ObjectBuffer
{
#define MAX_BLEND_MATRIX_COUNT 256
	matrix BlendMatrices[MAX_BLEND_MATRIX_COUNT];
#undef MAX_BLEND_MATRIX_COUNT
}



void Skin(inout VSInputNmTxWeights vin, uniform int boneCount)
{
	matrix skinning = 0;

	[unroll]
	for (int i = 0; i < boneCount; i++)
	{
		skinning += BlendMatrices[vin.Indices[i]] * vin.Weights[i];
	}

	vin.Position = mul(vin.Position, skinning);
}


float4 main( VSInputNmTxWeights input ) : SV_POSITION
{
	Skin(input,VERTEX_BLEND_BONES);
	return input.Position;
}