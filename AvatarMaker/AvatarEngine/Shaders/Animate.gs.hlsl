struct PixelInputType
{
    float4 position : SV_POSITION;
    float2 tex : TEXCOORD0;
	float3 normal : NORMAL;
	float3 viewDirection : TEXCOORD1;
};

[maxvertexcount(6)]
void BlenderNormalGeometryShader(triangle PixelInputType input[3],inout TriangleStream<PixelInputType> output)
{
	PixelInputType vout;
//	float3 v1,v2,n;
////	v1=input[1].position-input[0].position;
//	v1.x=input[1].position.x-input[0].position.x;
//	v1.y=input[1].position.y-input[0].position.y;
//	v1.z=input[1].position.z-input[0].position.z;
////	v2=input[1].position-input[0].position;
//	v2=input[2].position.x-input[0].position.x;
//	v2=input[2].position.y-input[0].position.y;
//	v2=input[2].position.z-input[0].position.z;
//
//	n=cross(v1,v2);
//	n = normalize(n);

	vout = input[0];
//	vout.normal = n;
//	vout.normal = normalize(vout.normal);
	output.Append(vout);

	vout = input[1];
//	vout.normal = n;
//	vout.normal = normalize(vout.normal);
	output.Append(vout);

	vout = input[2];
//	vout.normal = n;
//	vout.normal = normalize(vout.normal);
	output.Append(vout);

//	output.RestartStrip();
//////
//	vout = input[2];
//	vout.position = vout.position * 0.9;
//	vout.normal = - vout.normal;
//	output.Append(vout);
//
//	vout = input[1];
//	vout.position = vout.position * 0.9;
//	vout.normal = - vout.normal;
//	output.Append(vout);
//
//	vout = input[0];
//	vout.position = vout.position * 0.9;
//	vout.normal = - vout.normal;
//	output.Append(vout);

	//output.RestartStrip();

	//vout = input[0];
	//vout.normal = - vout.normal;
	//output.Append(vout);

	//vout = input[1];
	//vout.normal = - vout.normal;
	//output.Append(vout);

	//vout = input[2];
	//vout.normal = - vout.normal;
	//output.Append(vout);
}