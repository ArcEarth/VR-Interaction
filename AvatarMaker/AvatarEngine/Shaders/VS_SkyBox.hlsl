//
//cbuffer CamareBuffer
//{
//	matrix ViewProjectionMatrix;
//	float4 CameraPosition;
//	float4 CameraFocus;
//}

#include "Structures.hlsli"

cbuffer ObjectBuffer
{
	matrix WorldMatrix;
	matrix ViewMatrix;
	matrix ProjectionMatrix;
}

//struct VSInputTx
//{
//    float4 Position : SV_Position;
//    float2 TexCoord : TEXCOORD0;
//};

//struct PixelInputType
//{
//    float4 Position : SV_POSITION;
//    float2 TexCoord : TEXCOORD0;
//};
//

VSOutputNoFog SkyBoxVS(VSInputVc input)
{
	VSOutputNoFog output;
	output.PositionPS = mul(input.Position , WorldMatrix);
//	output.Position = mul(output.Position , ViewProjectionMatrix);
	output.PositionPS = mul(output.PositionPS , ViewMatrix);
	output.PositionPS = mul(output.PositionPS , ProjectionMatrix);
	output.Diffuse = input.Color;
//	output.TexCoord = input.TexCoord;
	return output;
}