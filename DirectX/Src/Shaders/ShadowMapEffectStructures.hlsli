
struct PSInputOneLightNoTex
{
	float4 pos : SV_POSITION;
	float3 normal : TEXCOORD2;
	float3 toEye : TEXCOORD4;
	float4 lightUv0 : TEXCOORD5;
};

struct PSInputTwoLightNoTex
{
	float4 pos : SV_POSITION;
	float3 normal : TEXCOORD2;
	float3 toEye : TEXCOORD4;
	float4 lightUv0 : TEXCOORD5;
	float4 lightUv1 : TEXCOORD6;
};

struct PSInputThreeLightNoTex
{
	float4 pos : SV_POSITION;
	float2 uv : TEXCOORD0;
	float3 normal : TEXCOORD2;
	float3 toEye : TEXCOORD4;
	float4 lightUv0 : TEXCOORD5;
	float4 lightUv1 : TEXCOORD6;
	float4 lightUv2 : TEXCOORD7;
};

struct PSInputFourLightNoTex
{
	float4 pos : SV_POSITION;
	float3 normal : TEXCOORD2;
	float3 toEye : TEXCOORD4;
	float4 lightUv0 : TEXCOORD5;
	float4 lightUv1 : TEXCOORD6;
	float4 lightUv2 : TEXCOORD7;
	float4 lightUv3 : TEXCOORD8;
};


struct PSInputOneLightTex
{
	float4 pos : SV_POSITION;
	float2 uv : TEXCOORD0;
	float3 normal : TEXCOORD2;
	float4 tangent : TEXCOORD3;
	float3 toEye : TEXCOORD4;
	float4 lightUv0 : TEXCOORD5;
};

struct PSInputTwoLightTex
{
	float4 pos : SV_POSITION;
	float2 uv : TEXCOORD0;
	float4 posWorld : TEXCOORD1;
	float3 normal : TEXCOORD2;
	float4 tangent : TEXCOORD3;
	float3 toEye : TEXCOORD4;
	float4 lightUv0 : TEXCOORD5;
	float4 lightUv1 : TEXCOORD6;
};

struct PSInputThreeLightTex
{
	float4 pos : SV_POSITION;
	float2 uv : TEXCOORD0;
	float4 posWorld : TEXCOORD1;
	float3 normal : TEXCOORD2;
	float4 tangent : TEXCOORD3;
	float3 toEye : TEXCOORD4;
	float4 lightUv0 : TEXCOORD5;
	float4 lightUv1 : TEXCOORD6;
	float4 lightUv2 : TEXCOORD7;
};

struct PSInputFourLightTex
{
	float4 pos : SV_POSITION;
	float2 uv : TEXCOORD0;
	float4 posWorld : TEXCOORD1;
	float3 normal : TEXCOORD2;
	float4 tangent : TEXCOORD3;
	float3 toEye : TEXCOORD4;
	float4 lightUv0 : TEXCOORD5;
	float4 lightUv1 : TEXCOORD6;
	float4 lightUv2 : TEXCOORD7;
	float4 lightUv3 : TEXCOORD8;
};
