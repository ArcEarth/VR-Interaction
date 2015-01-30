////////////////////
// GLOBALS BUFFERS//
////////////////////
// 
// This value is up to 1024 since each cbuffer can only hold 4096 vectors
// Shader model 4 can only support 256

#define MAX_BLEND_MATRIX_COUNT 255
#define VERTEX_BLEND_BONES 4
#define BrushCount 4

cbuffer ObjectBuffer : register(c0)
{
	matrix WorldMatrix;
	float4x3 BlendMatrices[MAX_BLEND_MATRIX_COUNT];
}

cbuffer BrushBuffer : register(c1)
{
	float4 BrushCenter[BrushCount]	: packoffset(c0);
	float4 BrushColor[BrushCount]	: packoffset(c4);
	uint  ActiveBrushCount			: packoffset(c8);
};


#include "Structures.hlsli"
#define NO_NORMAL_BLENDING
#include "Common.hlsli"


#define MODELING_FACTOR_BbyR 2.0f

inline float R2Function(float r2, float R)
{
	float b = MODELING_FACTOR_BbyR*R;
	float b2 = b*b;
	if (r2 <= b2) {
		if (r2 <= b2/9){
			return (1-3*r2/b2);
		}
		else
		{
			float r = sqrt(r2);
			float t = r / b;
			t = 1.0f - t;  
			t = t * t;
			t = 1.5f * t;
			return t;
		}
	} else
	{
		return 0.0f;
	}
}	

float4 BrushColorInPosition(in float3 Position , uniform int BrushIndex)
{
	// Calculate the intensity value
	float3 disp = Position - BrushCenter[BrushIndex].xyz;
	float r2 = dot(disp,disp);
	//float R2 = BrushCenter[BrushIndex].w*BrushCenter[BrushIndex].w;
	float intensity = R2Function(r2,BrushCenter[BrushIndex].w);
	/*if (r2 <= R2)
		intensity = 1.0f;
	else
		intensity = 0.0f;*/
	float4 color = BrushColor[BrushIndex];
		//R2Function(r2,BrushCenter[BrushIndex].w);
	color.w = intensity;
	return color;
}

VSOutputNoFog main( VSInputNmTxWeights input )
{
	VSOutputNoFog output;

	Skin(input,VERTEX_BLEND_BONES);

	input.TexCoord += float2(-0.5f,-0.5f);
	input.TexCoord *= float2(2.0f,-2.0f);

	float4 color = BrushColorInPosition(input.Position.xyz , 0);
	//[unroll]
	for (uint i = 1; i < ActiveBrushCount; i++)
	{
		float4 brushColor = BrushColorInPosition(input.Position.xyz , i);
		color = lerp(color,brushColor,brushColor.w);
	}

	output.Diffuse = color;

	output.PositionPS = float4(input.TexCoord.x,input.TexCoord.y,1.0f-output.Diffuse.w,1.0f);
	
	//output.PositionWS.w = 1.0f;
	return output;
}
