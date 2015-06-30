// A constant buffer that stores the three basic column-major matrices for composing geometry.
cbuffer ModelViewProjectionConstantBuffer : register(b0)
{
	matrix wvp;
};

float4 main( float4 pos : POSITION ) : SV_POSITION
{
	return pos;
}