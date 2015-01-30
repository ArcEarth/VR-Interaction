//Texture2D ColorTexture;
//SamplerState SampleType;

//struct PixelInputType
//{
//    float4 Position : SV_POSITION;
//    float2 TexCoord : TEXCOORD0;
//};

#include "Structures.hlsli"

float4 PosTexPS(PSInputNoFog input) : SV_TARGET
{
	return input.Diffuse;
}