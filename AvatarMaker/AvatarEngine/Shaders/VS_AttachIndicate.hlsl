////////////////////
// GLOBALS BUFFERS//
////////////////////
// 

#include "Structures.hlsli"

#define SphereLightCount 2
#define VERTEX_BLEND_BONES 4

cbuffer ObjectBuffer
{
	matrix WorldMatrix;
#define MAX_BLEND_MATRIX_COUNT 255
	float4x3 BlendMatrices[MAX_BLEND_MATRIX_COUNT];
#undef MAX_BLEND_MATRIX_COUNT
}

cbuffer CamareBuffer
{
	matrix ViewProjectionMatrix;
	float4 CameraPosition;
	float4 CameraFocus;
}


cbuffer LightBuffer
{
	float4 AmbientColor							: packoffset(c0);
	float4 DiffuseColor							: packoffset(c1);
	float3 LightDirection						: packoffset(c2);
	float4 SphereLightCenter[SphereLightCount]	: packoffset(c3);
	float4 SphereLightColor[SphereLightCount]	: packoffset(c5);
	float3 EyePosition							: packoffset(c7);
};


static const float4 BlendColors[20] = 
{
	{1.000000000f, 0.000000000f, 0.000000000f, 1.000000000f},	//HIP_C
	{1.000000000f, 0.647058845f, 0.000000000f, 1.000000000f},	//SPIN
	{1.000000000f, 1.000000000f, 0.000000000f, 1.000000000f},	//SHOULDER_C
	{0.678431392f, 1.000000000f, 0.184313729f, 1.000000000f},	//HEAD
	{0.486274540f, 0.988235354f, 0.000000000f, 1.000000000f},	//SHOULDER_L
	{0.529411793f, 0.807843208f, 0.921568692f, 1.000000000f},	//ELBOW_L
	{0.000000000f, 0.749019623f, 1.000000000f, 1.000000000f},	//WRIST_L
	{0.854902029f, 0.439215720f, 0.839215755f, 1.000000000f},	//HAND_L
	{0.486274540f, 0.988235354f, 0.000000000f, 1.000000000f},	//SHOULDER_R
	{0.529411793f, 0.807843208f, 0.921568692f, 1.000000000f},	//ELBOW_R
	{0.000000000f, 0.749019623f, 1.000000000f, 1.000000000f},	//WRIST_R
	{0.854902029f, 0.439215720f, 0.839215755f, 1.000000000f},	//HAND_R
	{1.000000000f, 0.647058845f, 0.000000000f, 1.000000000f},	//HIP_L
	{1.000000000f, 1.000000000f, 0.000000000f, 1.000000000f},	//KNEE_L
	{0.486274540f, 0.988235354f, 0.000000000f, 1.000000000f},	//ANKLE_L
	{0.529411793f, 0.807843208f, 0.921568692f, 1.000000000f},	//FOOT_L
	{1.000000000f, 0.647058845f, 0.000000000f, 1.000000000f},	//HIP_R
	{1.000000000f, 1.000000000f, 0.000000000f, 1.000000000f},	//KNEE_R
	{0.486274540f, 0.988235354f, 0.000000000f, 1.000000000f},	//ANKLE_R
	{0.529411793f, 0.807843208f, 0.921568692f, 1.000000000f},	//FOOT_R
};

void Skin(inout VSInputNmTxWeights vin, uniform int boneCount)
{
	float4x3 skinning = 0;

	[unroll]
	for (int i = 0; i < boneCount; i++)
	{
		skinning += BlendMatrices[vin.Indices[i]] * vin.Weights[i];
	}

	vin.Position.xyz = mul(vin.Position, skinning);
	vin.Normal = mul(vin.Normal, (float3x3)skinning);
}


VSOutputNoFog main( VSInputNmTxWeights input )
{    
	VSOutputNoFog output;

	Skin(input,VERTEX_BLEND_BONES);

	output.PositionPS = mul(input.Position,ViewProjectionMatrix);
	input.Normal = normalize(input.Normal);
	output.Diffuse = 0;
	[unroll]
	for (unsigned int i = 0; i < VERTEX_BLEND_BONES; i++)
	{
		output.Diffuse += input.Weights[i] * BlendColors[input.Indices[i]];
	}

	float DiffuseIntensity = saturate(dot(input.Normal , -LightDirection));
	float4 light = AmbientColor;
	light += DiffuseIntensity * DiffuseColor;
	light = saturate(light);

	output.Diffuse = output.Diffuse * light;

	return output;
}