// A constant buffer that stores the three basic column-major matrices for composing geometry.
cbuffer ModelViewProjectionConstantBuffer : register(b0)
{
	matrix WorldLightProj;
	float4x3 Bones[72];
};

struct VSInputWeights
{
	float4 Position : SV_Position;
	uint4  Indices  : BLENDINDICES0;
	float4 Weights  : BLENDWEIGHT0;
};

void Skin(inout VSInputWeights vin, uniform int boneCount)
{
	float4x3 skinning = 0;

	[unroll]
	for (int i = 0; i < boneCount; i++)
	{
		skinning += Bones[vin.Indices[i]] * vin.Weights[i];
	}

	vin.Position.xyz = mul(vin.Position, skinning);
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

float4 VS_NoBone(float4 pos : SV_Position) : SV_POSITION
{
	return mul(pos,WorldLightProj);
}

float4 PS(float4 pos : SV_POSITION) : SV_TARGET
{
	pos /= pos.w;
	return pos;
}