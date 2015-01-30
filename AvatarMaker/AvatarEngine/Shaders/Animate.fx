////////////////////////////////////////////////////////////////////////////////
// Filename: light.fx
////////////////////////////////////////////////////////////////////////////////

#define SKELETON_JOINT_COUNT 5

/////////////
// GLOBALS //
/////////////
matrix worldMatrix;
matrix viewMatrix;
matrix projectionMatrix;

matrix JointDisplacement[SKELETON_JOINT_COUNT];

Texture2D shaderTexture;
float4 ambientColor;
float4 diffuseColor;
float3 lightDirection;
//There are now three new global variables; cameraPosition, specularColor, and specularPower. These three variables are needed for specular light calculations.

float3 cameraPosition;
float4 specularColor;
float specularPower;


///////////////////
// SAMPLE STATES //
///////////////////
SamplerState SampleType
{
    Filter = MIN_MAG_MIP_LINEAR;
    AddressU = Wrap;
    AddressV = Wrap;
};


#define SKELETON_JOINT_COUNT 5

///////////////
//// GLOBALS //
///////////////
//cbuffer MatrixBuffer
//{
//	matrix worldMatrix;
//	matrix viewMatrix;
//	matrix projectionMatrix;
//};
//
//cbuffer CameraBuffer
//{
//    float3 cameraPosition;
//	float padding;
//};

//cbuffer SkeletonBuffer
//{
//	//Package the position vector 4 in 1
////	float padding[AnimatedModelClass % 4];
//}


//////////////
// TYPEDEFS //
//////////////
struct VertexInputType
{
    float4 position : POSITION;
    float2 tex : TEXCOORD0;
	float3 normal : NORMAL;
	float4 weight[SKELETON_JOINT_COUNT] : BLENDWEIGHT;
//	float  weight : BLENDWEIGHT0;
};

struct PixelInputType
{
    float4 position : SV_POSITION;
    float2 tex : TEXCOORD0;
	float3 normal : NORMAL;
	float3 viewDirection : TEXCOORD1;
};

////////////////////////////////////////////////////////////////////////////////
// Vertex Shader
////////////////////////////////////////////////////////////////////////////////
PixelInputType AnimateVertexShader(VertexInputType input)
{
    PixelInputType output;
	float4 worldPosition;
	int i;

//	 Calculate the position animated with the joint
	for(i=0;i<SKELETON_JOINT_COUNT;i++)
	{
		input.position = input.position + mul(input.weight[i],JointDisplacement[i]);
	}

	// Change the position vector to be 4 units for proper matrix calculations.
    input.position.w = 1.0f;

	// Calculate the position of the vertex against the world, view, and projection matrices.
    output.position = mul(input.position, worldMatrix);
    output.position = mul(output.position, viewMatrix);
    output.position = mul(output.position, projectionMatrix);
    
	// Store the texture coordinates for the pixel shader.
	output.tex = input.tex;
    
	// Calculate the normal vector against the world matrix only.
    output.normal = mul(input.normal, (float3x3)worldMatrix);
	
    // Normalize the normal vector.
    output.normal = normalize(output.normal);

	// Calculate the position of the vertex in the world.
    worldPosition = mul(input.position, worldMatrix);

    // Determine the viewing direction based on the position of the camera and the position of the vertex in the world.
    output.viewDirection = cameraPosition.xyz - worldPosition.xyz;
	
    // Normalize the viewing direction vector.
    output.viewDirection = normalize(output.viewDirection);

    return output;
}
//
[maxvertexcount(3)]
void AnimateGeometryShader(triangle PixelInputType input[3],inout TriangleStream<PixelInputType> output)
{
	float3 v1,v2,n;
	PixelInputType vout;
	v1=input[2].position-input[0].position;
	v2=input[1].position-input[0].position;
	n=cross(v1,v2);
	n = normalize(n);

	vout = input[0];
	vout.normal = n;
	output.Append(vout);

	vout = input[1];
	vout.normal = n;
	output.Append(vout);

	vout = input[2];
	vout.normal = n;
	output.Append(vout);
}

////////////////////////////////////////////////////////////////////////////////
// Pixel Shader
////////////////////////////////////////////////////////////////////////////////
float4 AnimatePixelShader(PixelInputType input) : SV_Target
{
    float4 textureColor;
    float3 lightDir;
    float lightIntensity;
    float4 color;
    float3 reflection;
    float4 specular;


    // Sample the pixel color from the texture using the sampler at this texture coordinate location.
    textureColor = shaderTexture.Sample(SampleType, input.tex);

    // Set the default output color to the ambient light value for all pixels.
    color = ambientColor;

    // Initialize the specular color.
    specular = float4(0.0f, 0.0f, 0.0f, 0.0f);

    // Invert the light direction for calculations.
    lightDir = -lightDirection;

    // Calculate the amount of light on this pixel.
    lightIntensity = saturate(dot(input.normal, lightDir));
	
    if(lightIntensity > 0.0f)
    {
        // Determine the final diffuse color based on the diffuse color and the amount of light intensity.
        color += (diffuseColor * lightIntensity);

        // Saturate the ambient and diffuse color.
        color = saturate(color);
//The reflection vector for specular lighting is calculated here in the pixel shader provided the light intensity is greater than zero. This is the same equation as listed at the beginning of the tutorial.

        // Calculate the reflection vector based on the light intensity, normal vector, and light direction.
        reflection = normalize(2 * lightIntensity * input.normal - lightDir); 
//The amount of specular light is then calculated using the reflection vector and the viewing direction. The smaller the angle between the viewer and the light source the greater the specular light reflection will be. The result is taken to the power of the specularPower value. The lower the specularPower value the greater the final effect is.

        // Determine the amount of specular light based on the reflection vector, viewing direction, and specular power.
        specular = pow(saturate(dot(reflection, input.viewDirection)), specularPower);
    }

    // Multiply the texture pixel and the input color to get the textured result.
    color = color * textureColor;
//We don't add the specular effect until the end. It is a highlight and needs to be added to the final value or it will not show up properly.

    // Add the specular component last to the output color.
    color = saturate(color + specular);

    return color;
}


////////////////////////////////////////////////////////////////////////////////
// Technique
////////////////////////////////////////////////////////////////////////////////
technique10 AnimatedLightTechnique
{
    pass pass0
    {
        SetVertexShader(CompileShader(vs_5_0, AnimateVertexShader()));
        SetGeometryShader(CompileShader(gs_5_0, AnimateGeometryShader()));
        SetPixelShader(CompileShader(ps_5_0, AnimatePixelShader()));
    }
}
