Texture2D DiffuseTex : register(t0);
Texture2D NormalTex : register(t1);
Texture2D ShadowTex : register(t2);

SamplerState DiffuseSampler : register(s0);
SamplerState NormalSampler : register(s1);
SamplerState ShadowSampler : register(s2);

#define REGISTER(b) : register(b)

#include "ShadowMapEffectCBuffer.hlsli"

#include "ShadowMapEffectStructures.hlsli"

static const float bias = 1e-4f;
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
	//float3 specular = 0;

	float4 shadowAmount = ShadowTex.Sample(ShadowSampler, pixel.posUV);
	[unroll]
	for (int i = 0; i < 1; i++)
	{
		diffuse += shadowAmount[i] * LambertLighting(LightDirection[i].xyz, worldNormal, LightColor[i].rgb, matDiffuse);
		//specular += SpecularContribution(toEyeVector, LightDirection[i], worldNormal, MaterialSpecular.rgb, MaterialSpecularPower, LightSpecularIntensity[i], LightColor[i].rgb);
	}
	diffuse = saturate(diffuse);

	return float4(diffuse, MaterialDiffuse.a);
}

float4 PS_ScreenSpaceTex(PSInputScreenSpaceTex pixel) : SV_TARGET
{
	float3 worldNormal = normalize(pixel.normal);
	float3 toEyeVector = normalize(pixel.toEye);

	float4 texDiffuse = DiffuseTex.Sample(DiffuseSampler, pixel.uv);
	float3 diffuse = MaterialAmbient.rgb * AmbientLight.rgb * texDiffuse.rgb;
	float3 matDiffuse = MaterialDiffuse.rgb * texDiffuse.rgb;

	float4 shadowAmount = ShadowTex.Sample(ShadowSampler, pixel.posUV);
	[unroll]
	for (int i = 0; i < 1; i++)
	{
		diffuse += shadowAmount[i] * LambertLighting(LightDirection[i].xyz, worldNormal, LightColor[i].rgb, matDiffuse);
		//specular += SpecularContribution(toEyeVector, LightDirection[i], worldNormal, MaterialSpecular.rgb, MaterialSpecularPower, LightSpecularIntensity[i], LightColor[i].rgb);
	}
	diffuse = saturate(diffuse);

	return float4(diffuse, MaterialDiffuse.a * texDiffuse.a);
}


float PS_BinaryOneLightNoTex(PSInputBinaryOneLightNoTex pixel) : SV_TARGET
{
	float amount;

	float shadowDepth = ShadowTex.Sample(ShadowSampler, pixel.lightUv0.xy).r;
	amount = (shadowDepth + bias > pixel.lightUv0.z) ? 1.0f : 0.0f;

	return amount;
}

float PS_BinaryOneLightTex(PSInputBinaryOneLightTex pixel) : SV_TARGET
{
	float4 texDiffuse = DiffuseTex.Sample(DiffuseSampler, pixel.uv);

	clip(texDiffuse.a - 0.15f);

	float amount;

	float shadowDepth = ShadowTex.Sample(ShadowSampler, pixel.lightUv0.xy).r;
	amount = (shadowDepth + bias > pixel.lightUv0.z) ? 1.0f : 0.0f;

	return amount;
}
