Texture2D Scene : register (t0);
SamplerState PointSampler : register (s0);

float4 main(float2 TexCoor : TEXCOOR0) : SV_TARGET
{
	float4 color = Scene.Sample(PointSampler, TexCoor);
	return color;
}