float2 EyeToSourceUVScale, EyeToSourceUVOffset;
float4x4 EyeRotationStart, EyeRotationEnd;

float2 TimewarpTexCoord(float2 TexCoord, float4x4 rotMat)
{
	// Vertex inputs are in TanEyeAngle space for the R,G,B channels (i.e. after chromatic 
	// aberration and distortion). These are now "real world" vectors in direction (x,y,1) 
	// relative to the eye of the HMD.	Apply the 3x3 timewarp rotation to these vectors.
	float3 transformed = float3(mul(rotMat, float4(TexCoord.xy, 1, 1)).xyz);
	// Project them back onto the Z=1 plane of the rendered images.
	float2 flattened = (transformed.xy / transformed.z);
	// Scale them into ([0,0.5],[0,1]) or ([0.5,1],[0,1]) UV lookup space (depending on eye)
	return(EyeToSourceUVScale * flattened + EyeToSourceUVOffset);
}

void main(	in float2  Position   : POSITION,	in float timewarpLerpFactor : POSITION1,
			in float   Vignette : POSITION2,	in float2 TexCoord0 : TEXCOORD0,
			in float2  TexCoord1 : TEXCOORD1,	in float2 TexCoord2 : TEXCOORD2,
			out float4 oPosition : SV_Position,
			out float2 oTexCoord0 : TEXCOORD0,	out float2 oTexCoord1 : TEXCOORD1,
			out float2 oTexCoord2 : TEXCOORD2,	out float  oVignette : TEXCOORD3)
{
	float4x4 lerpedEyeRot = lerp(EyeRotationStart, EyeRotationEnd, timewarpLerpFactor);
	oTexCoord0 = TimewarpTexCoord(TexCoord0, lerpedEyeRot);
	oTexCoord1 = TimewarpTexCoord(TexCoord1, lerpedEyeRot);
	oTexCoord2 = TimewarpTexCoord(TexCoord2, lerpedEyeRot);
	oPosition = float4(Position.xy, 0.5, 1.0);    oVignette = Vignette;
}
