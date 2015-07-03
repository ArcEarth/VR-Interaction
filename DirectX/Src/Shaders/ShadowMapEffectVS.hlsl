#define REGISTER(b) : register(b)

#include "ShadowMapEffectCBuffer.hlsli"

cbuffer SkinningBuffer : register (b1)
{
	float4x3 Bones[MAX_BONES];
};

#include "ShadowMapEffectStructures.hlsli"

struct VSInputNoTex
{
	float4 Position : SV_Position;
	float3 Normal	: NORMAL;
};

struct VSInputTex
{
	float4 Position : SV_Position;
	float3 Normal	: NORMAL;
	float2 TexCoord : TEXCOORD0;
};

struct VSInputNoTexWeights
{
	float4 Position : SV_Position;
	float3 Normal	: NORMAL;
	uint4  Indices  : BLENDINDICES0;
	float4 Weights  : BLENDWEIGHT0;
};

struct VSInputTexWeights
{
	float4 Position : SV_Position;
	float3 Normal	: NORMAL;
	float2 TexCoord : TEXCOORD0;
	uint4  Indices  : BLENDINDICES0;
	float4 Weights  : BLENDWEIGHT0;
};

#define SkinVertex \
	float4x3 skinning = 0; \
	[unroll] \
	for (int i = 0; i < boneCount; i++) \
		skinning += Bones[vin.Indices[i]] * vin.Weights[i]; \
	vin.Position.xyz = mul(vin.Position, skinning); \
	vin.Normal = mul(vin.Normal, (float3x3)skinning);


#define SetPositionNormalToEye \
	float4 posWorld = mul(vin.Position, World); \
	vout.pos = mul(posWorld, ViewProjection); \
	vout.normal = mul(vin.Normal, (float3x3)World); \
	vout.toEye = posWorld.xyz - EyePosition;


#define SetTextureCoord \
	vout.uv = vin.TexCoord;


#define SetLightUVOne \
	vout.lightUv0 = mul(posWorld, LightViewProjection[0]);


//#define SetLightUVTwo \ 
//	vout.lightUv0 = mul(posWorld, LightViewProjection[0]); \
//	vout.lightUv1 = mul(posWorld, LightViewProjection[1]);
//
//
//#define SetLightUVThree \ 
//	vout.lightUv0 = mul(posWorld, LightViewProjection[0]); \
//	vout.lightUv1 = mul(posWorld, LightViewProjection[1]); \
//	vout.lightUv2 = mul(posWorld, LightViewProjection[2]);
//
//
//#define SetLightUVTwo \ 
//	vout.lightUv0 = mul(posWorld, LightViewProjection[0]); \
//	vout.lightUv1 = mul(posWorld, LightViewProjection[1]); \
//	vout.lightUv2 = mul(posWorld, LightViewProjection[2]); \
//	vout.lightUv3 = mul(posWorld, LightViewProjection[3]);


void SkinVertexNoTex(inout VSInputNoTexWeights vin, uniform int boneCount)
{
	SkinVertex;
}

void SkinVertexTex(inout VSInputTexWeights vin, uniform int boneCount)
{
	SkinVertex;
}


PSInputOneLightNoTex VS_OneLightNoBoneNoTex(VSInputNoTex vin)
{
	PSInputOneLightNoTex vout;

	SetPositionNormalToEye;

	SetLightUVOne;

	return vout;
}

PSInputOneLightTex VS_OneLightNoBoneTex(VSInputTex vin)
{
	PSInputOneLightTex vout;

	SetPositionNormalToEye;

	SetTextureCoord;

	SetLightUVOne;

	return vout;
}

PSInputOneLightNoTex VS_OneLightOneBoneNoTex(VSInputNoTexWeights vin)
{
	PSInputOneLightNoTex vout;

	SkinVertexNoTex(vin, 1);

	SetPositionNormalToEye;

	SetLightUVOne;

	return vout;
}

PSInputOneLightTex VS_OneLightOneBoneTex(VSInputTexWeights vin)
{
	PSInputOneLightTex vout;

	SkinVertexTex(vin, 1);

	SetPositionNormalToEye;

	SetTextureCoord;

	SetLightUVOne;

	return vout;
}

PSInputOneLightNoTex VS_OneLightTwoBoneNoTex(VSInputNoTexWeights vin)
{
	PSInputOneLightNoTex vout;

	SkinVertexNoTex(vin, 2);

	SetPositionNormalToEye;

	SetLightUVOne;

	return vout;
}

PSInputOneLightTex VS_OneLightTwoBoneTex(VSInputTexWeights vin)
{
	PSInputOneLightTex vout;

	SkinVertexTex(vin, 2);

	SetPositionNormalToEye;

	SetTextureCoord;

	SetLightUVOne;

	return vout;
}

PSInputOneLightNoTex VS_OneLightFourBoneNoTex(VSInputNoTexWeights vin)
{
	PSInputOneLightNoTex vout;

	SkinVertexNoTex(vin, 4);

	SetPositionNormalToEye;

	SetLightUVOne;

	return vout;
}

PSInputOneLightTex VS_OneLightFourBoneTex(VSInputTexWeights vin)
{
	PSInputOneLightTex vout;

	SkinVertexTex(vin, 4);

	SetPositionNormalToEye;

	SetTextureCoord;

	SetLightUVOne;

	return vout;
}
