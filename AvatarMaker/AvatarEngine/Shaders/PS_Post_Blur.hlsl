Texture2D Scene : register (t0);
SamplerState PointSampler : register (s0);

#define BLUR_PIXEL_COUNT 5
#define BLUR_CENTRE_PIXEL 2

cbuffer cb0
{
    float2 SampleOffsets[BLUR_PIXEL_COUNT];
    float4 SampleWeights[BLUR_PIXEL_COUNT];
}

float4 main(float2 TexCoor : TEXCOOR0) : SV_TARGET
{
    float4 glow = 0.0f;
    float4 color = 0.0f;
    float2 samplePosition;

    for(int sampleIndex = 0; sampleIndex < BLUR_PIXEL_COUNT; sampleIndex++)
    {
        // Sample from adjacent points
        samplePosition = TexCoor + SampleOffsets[sampleIndex];
        color = Scene.Sample(PointSampler, samplePosition);

        glow += SampleWeights[sampleIndex] * color;
    }

    return glow;
}