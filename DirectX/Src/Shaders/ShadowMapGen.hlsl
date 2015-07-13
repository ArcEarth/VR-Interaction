// A constant buffer that stores the three basic column-major matrices for composing geometry.
cbuffer ModelViewProjectionConstantBuffer : register(b0)
{
	matrix WorldLightProj;
	float4 ShadowColor;
	float4x3 Bones[72];
};

Texture2D gDiffuseMap : register(t0);
SamplerState samLinear : register(s0);

struct VSInputTex
{
	float4 Position : SV_Position;
	float2 TexCoord : TEXCOORD0;
};

struct VSInputWeights
{
	float4 Position : SV_Position;
	uint4  Indices  : BLENDINDICES0;
	float4 Weights  : BLENDWEIGHT0;
};

struct VSInputTexWeights
{
	float4 Position : SV_Position;
	float2 TexCoord : TEXCOORD0;
	uint4  Indices  : BLENDINDICES0;
	float4 Weights  : BLENDWEIGHT0;
};

#include "Common.hlsli"

void Skin(inout VSInputWeights vin, uniform int boneCount)
{
	SkinVertex;
}

void SkinTex(inout VSInputTexWeights vin, uniform int boneCount)
{
	SkinVertex;
}

float4 VS_NoBone(float4 pos : SV_Position) : SV_POSITION
{
	return mul(pos,WorldLightProj);
}

float4 VS_OneBone(VSInputWeights vin) : SV_POSITION
{
	Skin(vin,1);
	return mul(vin.Position, WorldLightProj);
}

float4 VS_TwoBone(VSInputWeights vin) : SV_POSITION
{
	Skin(vin,2);
	return mul(vin.Position, WorldLightProj);
}

float4 VS_FourBone(VSInputWeights vin) : SV_POSITION
{
	Skin(vin,4);
	return mul(vin.Position, WorldLightProj);
}

VSInputTex VS_NoBoneTex(VSInputTex vin)
{
	VSInputTex vout;
	vout.Position = mul(vin.Position, WorldLightProj);
	vout.TexCoord = vin.TexCoord;
	return vout;
}

VSInputTex VS_OneBoneTex(VSInputTexWeights vin)
{
	VSInputTex vout;
	SkinTex(vin,1);
	vout.Position = mul(vin.Position, WorldLightProj);
	vout.TexCoord = vin.TexCoord;
	return vout;
}

VSInputTex VS_TwoBoneTex(VSInputTexWeights vin)
{
	VSInputTex vout;
	SkinTex(vin,2);
	vout.Position = mul(vin.Position, WorldLightProj);
	vout.TexCoord = vin.TexCoord;
	return vout;
}

VSInputTex VS_FourBoneTex(VSInputTexWeights vin)
{
	VSInputTex vout;
	SkinTex(vin,4);
	vout.Position = mul(vin.Position, WorldLightProj);
	vout.TexCoord = vin.TexCoord;
	return vout;
}

float PS_DepthNoTex(float4 pos : SV_POSITION) : SV_TARGET
{
	//pos /= pos.w;
	return pos.z;
}

float PS_DepthTex(VSInputTex pixel) : SV_TARGET
{
	float4 diffuse = gDiffuseMap.Sample(samLinear, pixel.TexCoord);
	clip(diffuse.a - 0.15f);
	//pixel.Position /= pixel.Position.w;
	return pixel.Position.z;
}

float4 PS_ColorNoTex(float4 pos : SV_POSITION) : SV_TARGET
{
	return float4(ShadowColor.rgb,pos.z);
}

float4 PS_ColorTex(VSInputTex pixel) : SV_TARGET
{
	float4 diffuse = gDiffuseMap.Sample(samLinear, pixel.TexCoord);
	clip(diffuse.a - 0.15f);
	return float4(ShadowColor.rgb, pixel.Position.z);
}