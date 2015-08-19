Texture2D DiffuseTex : register(t0);
Texture2D NormalTex : register(t1);
Texture2D ShadowTex : register(t2);
Texture2D ShadowTex2 : register(t3); //Unblured version of screen space shadow map

SamplerState DiffuseSampler : register(s0);
SamplerState NormalSampler : register(s1);
SamplerState ShadowSampler : register(s2);

#define REGISTER(b) : register(b)

#include "ShadowMapEffectCBuffer.hlsli"

#include "ShadowMapEffectStructures.hlsli"

static const float bias = 1e-4f;
static const float spbias = 0.003f;

//
// lambert lighting function
//
float3 LambertLighting(
	float3 lightNormal,
	float3 surfaceNormal,
	float3 lightColor,
	float3 pixelColor
	)
{
	// compute amount of contribution per light
	float diffuseAmount = saturate(dot(lightNormal, surfaceNormal));
	float3 diffuse = diffuseAmount * lightColor * pixelColor;
	return diffuse;
}

//
// specular contribution function
//
float3 SpecularContribution(
	float3 toEye,
	float3 lightNormal,
	float3 surfaceNormal,
	float3 materialSpecularColor,
	float  materialSpecularPower,
	float  lightSpecularIntensity,
	float3 lightColor)
{
	// compute specular contribution
	float3 vHalf = normalize(lightNormal + toEye);
	float specularAmount = saturate(dot(surfaceNormal, vHalf));
	specularAmount = pow(specularAmount, max(materialSpecularPower, 0.0001f)) * lightSpecularIntensity;
	float3 specular = materialSpecularColor * lightColor * specularAmount;

	return specular;
}

//float SampleShadow(Texture2D<float> tex, float2 uv, float depth)
//{
//	float shadowAmount = tex.SampleCmp(ShadowSampler, uv, depth, int2(-1, -1));
//	shadowAmount += tex.SampleCmp(ShadowSampler, uv, depth, int2(1, -1));
//	shadowAmount += tex.SampleCmp(ShadowSampler, uv, depth, int2(-1, 1));
//	shadowAmount += tex.SampleCmp(ShadowSampler, uv, depth, int2(1, 1));
//	shadowAmount /= 4.0f;
//	return shadowAmount;
//}

float4 PS_OneLightNoTex(PSInputOneLightNoTex pixel) : SV_TARGET
{
	float3 worldNormal = normalize(pixel.normal);
	float3 toEyeVector = normalize(pixel.toEye);

	float3 diffuse = MaterialAmbient.rgb * AmbientLight.rgb;
	float3 matDiffuse = MaterialDiffuse.rgb;
	//float3 specular = 0;

	[unroll]
	for (int i = 0; i < 1; i++)
	{
		float shadowDepth = ShadowTex.Sample(ShadowSampler, pixel.lightUv0.xy).r;
		//SampleShadow(ShadowTex, pixel.lightUv0.z - bias, pixel.lightUv0.xy);
		if (shadowDepth + bias > pixel.lightUv0.z)
		{
			diffuse += LambertLighting(LightDirection[i].xyz, worldNormal, LightColor[i].rgb, matDiffuse);
			//specular += SpecularContribution(toEyeVector, LightDirection[i], worldNormal, MaterialSpecular.rgb, MaterialSpecularPower, LightSpecularIntensity[i], LightColor[i].rgb);
		}
	}
	diffuse = saturate(diffuse);

	return float4(diffuse, MaterialDiffuse.a);
}

float4 PS_OneLightTex(PSInputOneLightTex pixel) : SV_TARGET
{
	float3 worldNormal = normalize(pixel.normal);
	float3 toEyeVector = normalize(pixel.toEye);

	float4 texDiffuse = DiffuseTex.Sample(DiffuseSampler, pixel.uv);
	float3 diffuse = MaterialAmbient.rgb * AmbientLight.rgb * texDiffuse.rgb;
	float3 matDiffuse = MaterialDiffuse.rgb * texDiffuse.rgb;

	//float3 specular = 0;

	[unroll]
	for (int i = 0; i < 1; i++)
	{
		float shadowDepth = ShadowTex.Sample(ShadowSampler, pixel.lightUv0.xy).r;
		if (shadowDepth + bias > pixel.lightUv0.z)
		{
			diffuse += LambertLighting(LightDirection[i].xyz, worldNormal, LightColor[i].rgb, matDiffuse);
			//specular += SpecularContribution(toEyeVector, LightDirection[i], worldNormal, MaterialSpecular.rgb, MaterialSpecularPower, LightSpecularIntensity[i], LightColor[i].rgb);
		}
	}
	diffuse = saturate(diffuse);

	return float4(diffuse, MaterialDiffuse.a * texDiffuse.a);
}


float4 PS_ScreenSpaceNoTex(PSInputScreenSpaceNoTex pixel) : SV_TARGET
{
	float3 worldNormal = normalize(pixel.normal);
	float3 toEyeVector = normalize(pixel.toEye);

	float3 diffuse = MaterialAmbient.rgb * AmbientLight.rgb;
	float3 matDiffuse = MaterialDiffuse.rgb;
	float3 specular = 0;

	//float2 pixeluv = pixel.pos.xy * float2(0.5f, -0.5f) + 0.5f;
	//float4 shadowAmount = ShadowTex.Sample(ShadowSampler, pixel.posUV.xy);
	float4 shadowAmount = ShadowTex.Load(pixel.pos);
	//shadowAmount = pixel.pos.z > shadowAmount.w - spbias ? shadowAmount : 1.0f;//ShadowTex2.Load(pixel.pos);

	[unroll]
	for (int i = 0; i < 1; i++)
	{
		diffuse += shadowAmount[i] * LambertLighting(LightDirection[i].xyz, worldNormal, LightColor[i].rgb, matDiffuse);
		specular += shadowAmount[i] * SpecularContribution(toEyeVector, LightDirection[i].xyz, worldNormal, MaterialSpecular.rgb, MaterialSpecularPower, LightColor[i].a, LightColor[i].rgb);
	}
	diffuse = saturate(diffuse + specular);

	return float4(diffuse, MaterialDiffuse.a);
}

float4 PS_ScreenSpaceTex(PSInputScreenSpaceTex pixel) : SV_TARGET
{
	float4 texDiffuse = DiffuseTex.Sample(DiffuseSampler, pixel.uv);

	clip(texDiffuse.a - 0.15);

	float3 worldNormal = normalize(pixel.normal);
	float3 toEyeVector = normalize(pixel.toEye);


	float3 diffuse = MaterialAmbient.rgb * AmbientLight.rgb * texDiffuse.rgb;
	float3 matDiffuse = MaterialDiffuse.rgb * texDiffuse.rgb;

	float4 shadowAmount = ShadowTex.Load(pixel.pos);
	//shadowAmount = pixel.pos.z > shadowAmount.w - spbias ? shadowAmount : 1.0f;// ShadowTex2.Load(pixel.pos);
	//shadowAmount = abs(pixel.pos.z - shadowAmount.w) < spbias ? shadowAmount : 1.0f;
	float3 specular = .0f;

	[unroll]
	for (int i = 0; i < 1; i++)
	{
		diffuse += shadowAmount[i] * LambertLighting(LightDirection[i].xyz, worldNormal, LightColor[i].rgb, matDiffuse);
		specular += shadowAmount[i] * SpecularContribution(toEyeVector, LightDirection[i].xyz, worldNormal, MaterialSpecular.rgb, MaterialSpecularPower, LightColor[i].a, LightColor[i].rgb);
	}
	diffuse = saturate(diffuse + specular);

	return float4(diffuse, MaterialDiffuse.a * texDiffuse.a);
}

float4 PS_ScreenSpaceTexBump(PSInputScreenSpaceTex pixel) : SV_TARGET
{
	float4 texDiffuse = DiffuseTex.Sample(DiffuseSampler, pixel.uv);

	clip(texDiffuse.a - 0.15);

	// Sample the pixel in the bump map.
	float2 texBump = NormalTex.Sample(NormalSampler, pixel.uv);
	texBump = (texBump * 2.0f) - 1.0f;
	// Calculate the normal from the data in the bump map.
	float3 worldNormal = pixel.normal + texBump.x * pixel.tangent + texBump.y * pixel.binormal;
	worldNormal = normalize(worldNormal);

	float3 toEyeVector = normalize(pixel.toEye);


	float3 diffuse = MaterialAmbient.rgb * AmbientLight.rgb * texDiffuse.rgb;
	float3 matDiffuse = MaterialDiffuse.rgb * texDiffuse.rgb;

	float4 shadowAmount = ShadowTex.Load(pixel.pos);
	//shadowAmount = pixel.pos.z > shadowAmount.w - spbias ? shadowAmount : 1.0f;// ShadowTex2.Load(pixel.pos);
	float3 specular = .0f;																	   //shadowAmount = abs(pixel.pos.z - shadowAmount.w) < spbias ? shadowAmount : 1.0f;

	[unroll]
	for (int i = 0; i < 1; i++)
	{
		diffuse += shadowAmount[i] * LambertLighting(LightDirection[i].xyz, worldNormal, LightColor[i].rgb, matDiffuse);
		specular += shadowAmount[i] * SpecularContribution(toEyeVector, LightDirection[i].xyz, worldNormal, MaterialSpecular.rgb, MaterialSpecularPower, LightColor[i].a, LightColor[i].rgb);
	}
	diffuse = saturate(diffuse + specular);

	return float4(diffuse, MaterialDiffuse.a * texDiffuse.a);
}


float4 PS_BinaryOneLightNoTex(PSInputBinaryOneLightNoTex pixel) : SV_TARGET
{
	float4 amount;
	amount.w = pixel.pos.z;

	[unroll]
	for (int i = 0; i < 1; i++)
	{
		float shadowDepth = ShadowTex.Sample(ShadowSampler, pixel.lightUv[i].xy).r;
		amount[i] = (shadowDepth + bias > pixel.lightUv[i].z) ? 1.0f : 0.0f;
	}

	//clip(0.85 - amount.r);

	return amount;
}

float4 PS_BinaryOneLightTex(PSInputBinaryOneLightTex pixel) : SV_TARGET
{
	float4 texDiffuse = DiffuseTex.Sample(DiffuseSampler, pixel.uv);

	clip(texDiffuse.a - 0.15f); // Sample Alpha clip texture
	float alpha = texDiffuse.a;

	float4 amount;
	amount.w = pixel.pos.z;

	[unroll]
	for (int i = 0; i < 1; i++)
	{
		float shadowDepth = ShadowTex.Sample(ShadowSampler, pixel.lightUv[i].xy).r;
		amount[i] = (shadowDepth + bias > pixel.lightUv[i].z) ? alpha : 0.0f;
	}

	//clip(0.85 - amount.r);

	return amount;
}
