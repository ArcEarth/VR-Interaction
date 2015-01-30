#pragma once

#include <DirectXMath.h>
#include <cstring>
#include <DirectXPackedVector.h>

namespace D3D10Utils 
{
	// Note: Make sure structure alignment agrees with HLSL structure padding rules. 
	// Elements are packed into 4D vectors with the restriction that an element
	// cannot straddle a 4D vector boundary.

	struct DirectionalLight
	{
		__forceinline DirectionalLight();

		DirectX::XMFLOAT4A AmbientColor;
		DirectX::XMFLOAT4A DiffuseColor;
		DirectX::XMFLOAT4A SpecularColor;
		DirectX::XMFLOAT3A Direction;
	};

	DirectionalLight:: DirectionalLight()
	{
		memset(this, 0, sizeof(DirectionalLight));
	}  

	struct PointLight
	{
		__forceinline PointLight();

		DirectX::XMFLOAT4A AmbientColor;
		DirectX::XMFLOAT4A DiffuseColor;
		DirectX::XMFLOAT4A SpecularColor;

		DirectX::XMFLOAT3A Attenuation;

		DirectX::XMFLOAT3 Position;
		float Range;
	};

	PointLight:: PointLight()
	{
		memset(this, 0, sizeof(PointLight)); 
	} 

	struct SpotLight
	{
		__forceinline SpotLight();

		DirectX::XMFLOAT4A AmbientColor;
		DirectX::XMFLOAT4A DiffuseColor;
		DirectX::XMFLOAT4A SpecularColor;

		DirectX::XMFLOAT3A Direction;

		DirectX::XMFLOAT3 Position;
		float Range;

		DirectX::XMFLOAT3 Attenuation;
		float SpotFactor;
	};

	SpotLight:: SpotLight()
	{
		memset(this, 0, sizeof(SpotLight));
	} 	
}

