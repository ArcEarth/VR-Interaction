#include "Structures.hlsli"

float4 main(PSInputNoFog input) : SV_TARGET
{
	return input.Diffuse;
}