
struct PSInputOneLightNoTex
{
	float4 pos : SV_POSITION;
	float3 normal : TEXCOORD0;
	float3 toEye : TEXCOORD1;
	float4 lightUv0 : TEXCOORD2;
};

struct PSInputTwoLightNoTex
{
	float4 pos : SV_POSITION;
	float3 normal : TEXCOORD0;
	float3 toEye : TEXCOORD1;
	float4 lightUv0 : TEXCOORD2;
	float4 lightUv1 : TEXCOORD3;
};

struct PSInputThreeLightNoTex
{
	float4 pos : SV_POSITION;
	float2 uv : TEXCOORD0;
	float3 normal : TEXCOORD1;
	float3 toEye : TEXCOORD2;
	float4 lightUv0 : TEXCOORD3;
	float4 lightUv1 : TEXCOORD4;
	float4 lightUv2 : TEXCOORD5;
};

struct PSInputFourLightNoTex
{
	float4 pos : SV_POSITION;
	float3 normal : TEXCOORD0;
	float3 toEye : TEXCOORD1;
	float4 lightUv0 : TEXCOORD2;
	float4 lightUv1 : TEXCOORD3;
	float4 lightUv2 : TEXCOORD4;
	float4 lightUv3 : TEXCOORD5;
};


struct PSInputOneLightTex
{
	float4 pos : SV_POSITION;
	float2 uv : TEXCOORD0;
	float3 normal : TEXCOORD1;
	float4 tangent : TEXCOORD2;
	float3 toEye : TEXCOORD3;
	float4 lightUv0 : TEXCOORD4;
};

struct PSInputTwoLightTex
{
	float4 pos : SV_POSITION;
	float2 uv : TEXCOORD0;
	float3 normal : TEXCOORD1;
	float4 tangent : TEXCOORD2;
	float3 toEye : TEXCOORD3;
	float4 lightUv0 : TEXCOORD4;
	float4 lightUv1 : TEXCOORD5;
};

struct PSInputThreeLightTex
{
	float4 pos : SV_POSITION;
	float2 uv : TEXCOORD0;
	float3 normal : TEXCOORD1;
	float4 tangent : TEXCOORD2;
	float3 toEye : TEXCOORD3;
	float4 lightUv0 : TEXCOORD4;
	float4 lightUv1 : TEXCOORD5;
	float4 lightUv2 : TEXCOORD6;
};

struct PSInputFourLightTex
{
	float4 pos : SV_POSITION;
	float2 uv : TEXCOORD0;
	float3 normal : TEXCOORD1;
	float4 tangent : TEXCOORD2;
	float3 toEye : TEXCOORD3;
	float4 lightUv0 : TEXCOORD4;
	float4 lightUv1 : TEXCOORD5;
	float4 lightUv2 : TEXCOORD6;
	float4 lightUv3 : TEXCOORD7;
};

struct PSInputBinaryOneLightNoTex
{
	float4 pos : SV_POSITION;
	float4 lightUv[1] : TEXCOORD0;
};

struct PSInputBinaryTwoLightNoTex
{
	float4 pos : SV_POSITION;
	float4 lightUv[2] : TEXCOORD0;
};

struct PSInputBinaryThreeLightNoTex
{
	float4 pos : SV_POSITION;
	float4 lightUv[3] : TEXCOORD0;
};

struct PSInputBinaryFourLightNoTex
{
	float4 pos : SV_POSITION;
	float4 lightUv[4] : TEXCOORD0;
};


struct PSInputBinaryOneLightTex
{
	float4 pos : SV_POSITION;
	float2 uv : TEXCOORD0;
	float4 lightUv[1] : TEXCOORD1;
};

struct PSInputBinaryTwoLightTex
{
	float4 pos : SV_POSITION;
	float2 uv : TEXCOORD0;
	float4 lightUv[2] : TEXCOORD1;
};

struct PSInputBinaryThreeLightTex
{
	float4 pos : SV_POSITION;
	float2 uv : TEXCOORD0;
	float4 lightUv[3] : TEXCOORD0;
};

struct PSInputBinaryFourLightTex
{
	float4 pos : SV_POSITION;
	float2 uv : TEXCOORD0;
	float4 lightUv[4] : TEXCOORD0;
};

struct PSInputScreenSpaceNoTex
{
	float4 pos : SV_POSITION;
	float4 posUV : TEXCOORD0;
	float3 normal : TEXCOORD1;
	float3 toEye : TEXCOORD2;
};

struct PSInputScreenSpaceTex
{
	float4 pos			: SV_POSITION;
	float2 uv			: TEXCOORD0;
	float4 posUV        : TEXCOORD1;
	float3 normal       : TEXCOORD2;
	float3 toEye        : TEXCOORD3;
	float3 tangent      : TEXCOORD4;
	float3 binormal     : TEXCOORD5;
};
