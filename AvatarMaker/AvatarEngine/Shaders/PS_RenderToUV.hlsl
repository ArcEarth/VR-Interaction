#define BrushCount 4
#define INVSQRT_2PI 0.39894228040143

cbuffer BrushBuffer : register(c0)
{
	float4 BrushCenter[BrushCount]	: packoffset(c0);
	float4 BrushColor[BrushCount]	: packoffset(c4);
};

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
			float t = r / b;
			t = 1.0f - t;  
			t = t * t;
			t = 1.5f * t;
			return t;
		}
	} else
	{
		return 0.0f;
	}
}	

float4 BrushColorInPosition(in float3 Position , uniform int BrushIndex)
{
	// Calculate the intensity value
	float3 disp = Position - BrushCenter[BrushIndex].xyz;
	float r2 = dot(disp,disp);
	//float R2 = BrushCenter[BrushIndex].w*BrushCenter[BrushIndex].w;
	float intensity = R2Function(r2,BrushCenter[BrushIndex].w);
	/*if (r2 <= R2)
		intensity = 1.0f;
	else
		intensity = 0.0f;*/
	float4 color = BrushColor[BrushIndex];
		//R2Function(r2,BrushCenter[BrushIndex].w);
	color.w = intensity;
	return color;
}

struct PsInputUVTarget
{
	float3 PositionWS	: TEXCOORD0;
};


float4 main(PsInputUVTarget input) : SV_TARGET
{
	//[unroll]
	//for (int i = 0; i < BrushCount; i++)
	//{
	//	float4 brushColor = BrushColorInPosition(input.PositionWS.xyz , i);
	//	color.xyz *= 1.0f - brushColor.w;
	//	color.xyz += brushColor.xyz * brushColor.w;
	//	color.w += brushColor.w;
	//}

	//float4 color = BrushColorInPosition(input.PositionWS, 0);
	//return color;
	return float4(input.PositionWS,1.0f);
}
