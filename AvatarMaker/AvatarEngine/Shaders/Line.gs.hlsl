struct PixelInputType
{
    float4 position : SV_POSITION;
    float4 color : COLOR;
};

[maxvertexcount(4)]
void WideLineGeometryShader(
	line PixelInputType input[2], 
	inout TriangleStream< PixelInputType > output
)
{
	float4 n = input[1].position - input[0].position;
	n.z = -n.x; n.x = n.y; n.y = n.z;
	n.z = 0.0f;	n.w = 0.0f;
	n = normalize(n)*0.02f;
	PixelInputType e;
	e.position = input[0].position + n;
	e.color = input[0].color;
	output.Append(e);
	e.position = input[0].position - n;
	output.Append(e);
	e.position = input[1].position + n; 
	e.color = input[1].color;
	output.Append(e);
	e.position = input[1].position - n;
	output.Append(e);
}