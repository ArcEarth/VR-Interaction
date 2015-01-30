////////////////////////////////////////////////////////////////////////////////
// Filename: light.ps
////////////////////////////////////////////////////////////////////////////////
#include "Structures.hlsli"


#define SphereLightCount 4
#define INVSQRT_2PI 0.39894228040143

/////////////
// GLOBALS //
/////////////
Texture2D ColorTexture;
SamplerState SampleType;

cbuffer LightBuffer
{
	float4 AmbientColor							: packoffset(c0);
	float4 DiffuseColor							: packoffset(c1);
	float3 LightDirection						: packoffset(c2);
	float4 SphereLightCenter[SphereLightCount]	: packoffset(c3);
	float4 SphereLightColor[SphereLightCount]	: packoffset(c7);
	float3 EyePosition							: packoffset(c11);
};


//////////////
// TYPEDEFS //
//////////////
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
			//float r = DirectX::sqrtEst2(r2);
			float t = r / b;
			t = 1.0f - t;  //This statement cost even more than sqrt......
			t = t * t;
			t = 1.5f * t;
			return t;
		}
	} else
	{
		return 0.0f;
	}
}	


void SphereHighLight(inout float4 Color , in float3 Position , uniform int LightIndex)
{
	if (SphereLightColor[LightIndex].w == 0.0f) return;
	// Caculate the Guass value
	float3 disp = Position - SphereLightCenter[LightIndex].xyz;
	float r2 = dot(disp,disp);
	float intensity = R2Function(r2,SphereLightCenter[LightIndex].w);

	//r2 *= -0.5 * SphereLightCenter[LightIndex].w * SphereLightCenter[LightIndex].w;
	//float intensity = exp(r2) * INVSQRT_2PI * SphereLightCenter[LightIndex].w;

	//float intensity = saturate(1.0f-r2);
	float4 highlight = intensity * SphereLightColor[LightIndex];

	Color *= 1-intensity;
	Color += highlight;
	Color = saturate(Color);

	//Color.xyz = float3(r2,r2,r2);
}


#define specularPower 64.0f
////////////////////////////////////////////////////////////////////////////////
// Pixel Shader
////////////////////////////////////////////////////////////////////////////////
float4 PixelLightPS(PSInputPixelLightingTx input) : SV_TARGET
{
	//return normalize(input.Diffuse);
	float4 color;
	float4 textureColor;
	float3 lightDir;
	float  lightIntensity;
	float3 reflection;
	float3 specular;

	// Sample the pixel color from the texture using the sampler at this texture coordinate location.
	textureColor = ColorTexture.Sample(SampleType, input.TexCoord);

	// Set the default output color to the ambient light value for all pixels.
	color = AmbientColor;

	// Initialize the specular color.
	specular = float3(0.0f, 0.0f, 0.0f);

	// Invert the light direction for calculations.
	lightDir = -LightDirection;

	input.NormalWS = normalize(input.NormalWS);

	// Calculate the amount of light on this pixel.
	lightIntensity = saturate(dot(input.NormalWS, lightDir));

	if(lightIntensity > 0.0f)
	{
		// Determine the final diffuse color based on the diffuse color and the amount of light intensity.
		color.xyz += (DiffuseColor.xyz * lightIntensity);

		// Calculate the reflection vector based on the light intensity, normal vector, and light direction.
		reflection = normalize(2 * lightIntensity * input.NormalWS - lightDir); 

		float3 viewDirection = EyePosition - input.PositionWS.xyz;
		viewDirection = normalize(viewDirection);
		// Determine the amount of specular light based on the reflection vector, viewing direction, and specular power.
		specular = pow(saturate(dot(reflection, viewDirection)), specularPower);
	}
	color = saturate(color);


	// Multiply the texture pixel and the input color to get the textured result.
	color = color * textureColor;

	[unroll]
	for (int i = 0; i < SphereLightCount; i++)
	{
		SphereHighLight(color , input.PositionWS.xyz , i);
	}

	// Add the specular component last to the output color.
	color.xyz += specular;
	color = saturate(color);



	return color;
}
