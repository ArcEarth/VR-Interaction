Texture2D Scene0 : register(t0);
Texture2D Scene1 : register(t1);
SamplerState Sampler0 : register(s0);
SamplerState Sampler1 : register(s1);


float4 main(float2 TexCoor : TEXCOOR0) : SV_TARGET
{
	float4 color = Scene0.Sample(Sampler0 , TexCoor);
	color += Scene1.Sample(Sampler1 , TexCoor);
	return color;
}