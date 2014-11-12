Texture2D Texture   : register(t0);
SamplerState Linear : register(s0);
float4 main(in float4 oPosition  : SV_Position, in float2 oTexCoord0 : TEXCOORD0,
			in float2 oTexCoord1 : TEXCOORD1,	in float2 oTexCoord2 : TEXCOORD2,
			in float  oVignette : TEXCOORD3) : SV_Target
{
	// 3 samples for fixing chromatic aberrations
	float R = Texture.Sample(Linear, oTexCoord0.xy).r;
	float G = Texture.Sample(Linear, oTexCoord1.xy).g;
	float B = Texture.Sample(Linear, oTexCoord2.xy).b;
	return (oVignette*float4(R, G, B, 1));
};