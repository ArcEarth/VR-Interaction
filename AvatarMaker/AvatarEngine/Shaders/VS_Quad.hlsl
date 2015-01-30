struct VSOutputPost
{
	float2 TexCoord   : TEXCOORD0;
	float4 PositionPS : SV_Position;
};

VSOutputPost main(uint VertexIndex : SV_VertexID){
	const float2 corners[4] = {
		float2(-1.0f, 1.0f),
		float2( 1.0f, 1.0f),
		float2(-1.0f,-1.0f),
		float2( 1.0f,-1.0f)
	};
	const float2 conersUV[4] = 
	{
		float2( 0.0f, 0.0f),
		float2( 1.0f, 0.0f),
		float2( 0.0f, 1.0f),
		float2( 1.0f, 1.0f)
	};

	VSOutputPost Output;
	
	Output.PositionPS = float4(corners[VertexIndex].xy, 0.0f, 1.0f);
	Output.TexCoord = conersUV[VertexIndex];
	return Output;
};