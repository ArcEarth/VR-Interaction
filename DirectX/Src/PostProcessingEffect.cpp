#include "pch_directX.h"
#include "PostProcessingEffect.h"
#include "SharedResourcePool.h"
#include "DemandCreate.h"
#include "ConstantBuffer.h"
#include "Models.h"
#include <VertexTypes.h>

using namespace DirectX;
using namespace DirectX::HLSLVectors;
// Points to a precompiled vertex or pixel shader program.
struct ShaderBytecode
{
	void const* code;
	size_t length;
};

using Microsoft::WRL::ComPtr;

struct QuadVertexShaderConstant
{
	float2 uv_base;
	float2 uv_range;
};

namespace DirectX
{
	namespace EffectDirtyFlags
	{
		static const int VSConstantBuffer = 0x400;
		static const int PSConstantBuffer = 0x800;
	}
}

// Factory for lazily instantiating shaders. BasicEffect supports many different
// shader permutations, so we only bother creating the ones that are actually used.
class PostEffectDeviceResources
{
private:
	ID3D11VertexShader* DemandCreateVertexShader(_Inout_ Microsoft::WRL::ComPtr<ID3D11VertexShader>& vertexShader, ShaderBytecode const& bytecode);

public:
	PostEffectDeviceResources(_In_ ID3D11Device* device)
		: m_Device(device)
	{
	}

	// Gets or lazily creates the specified vertex shader permutation.
	ID3D11VertexShader* GetVertexShader()
	{
		return DemandCreateVertexShader(m_QuadVertexShader, s_QuadVertexShaderBtyeCode);
	}

	ConstantBuffer<QuadVertexShaderConstant>& GetVSCbuffer() { return m_VSCBuffer; }

	ID3D11PixelShader * DemandCreatePixelShader(_Inout_ Microsoft::WRL::ComPtr<ID3D11PixelShader> & pixelShader, ShaderBytecode const& bytecode);

	ID3D11ShaderResourceView* GetDefaultTexture();

	void DrawQuad(ID3D11DeviceContext* pContext)
	{
		assert(!m_QuadMesh.Empty());
		m_QuadMesh.Draw(pContext);
	}

	static ShaderBytecode s_QuadVertexShaderBtyeCode;
protected:
	Microsoft::WRL::ComPtr<ID3D11Device>			 m_Device;
	Microsoft::WRL::ComPtr<ID3D11VertexShader>		 m_QuadVertexShader;
	Scene::MeshBuffer								 m_QuadMesh;
	Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> m_DefaultTexture;
	ConstantBuffer<QuadVertexShaderConstant>		 m_VSCBuffer;

	std::mutex										 m_Mutex;

};

static VertexPositionTexture QuadVertices[] = {
	{{ -1.0f,  1.0f, 0.5f, 1.0f }, { 0.0f, 0.0f }},
	{{ 1.0f ,  1.0f, 0.5f, 1.0f }, { 1.0f, 0.0f }},
	{{ -1.0f, -1.0f, 0.5f, 1.0f }, { 0.0f, 1.0f }},
	{{ 1.0f , -1.0f, 0.5f, 1.0f }, { 1.0f, 1.0f }}
};

// Gets or lazily creates the specified vertex shader permutation.
ID3D11VertexShader* PostEffectDeviceResources::DemandCreateVertexShader(_Inout_ ComPtr<ID3D11VertexShader>& vertexShader, ShaderBytecode const& bytecode)
{
	return DemandCreate(vertexShader, m_Mutex, [&](ID3D11VertexShader** pResult) -> HRESULT
	{
		HRESULT hr = m_Device->CreateVertexShader(bytecode.code, bytecode.length, nullptr, pResult);

		if (SUCCEEDED(hr))
			SetDebugObjectName(*pResult, "DirectXTK:PostEffectQuadVertexShader");

		m_QuadMesh.CreateDeviceResources<VertexPositionTexture>(m_Device.Get(), QuadVertices, std::size(QuadVertices), nullptr, D3D_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP);
		m_QuadMesh.CreateInputLayout(m_Device.Get(), s_QuadVertexShaderBtyeCode.code, s_QuadVertexShaderBtyeCode.length);
		return hr;
	});
}


// Gets or lazily creates the specified pixel shader permutation.
ID3D11PixelShader* PostEffectDeviceResources::DemandCreatePixelShader(_Inout_ ComPtr<ID3D11PixelShader>& pixelShader, ShaderBytecode const& bytecode)
{
	return DemandCreate(pixelShader, m_Mutex, [&](ID3D11PixelShader** pResult) -> HRESULT
	{
		HRESULT hr = m_Device->CreatePixelShader(bytecode.code, bytecode.length, nullptr, pResult);

		if (SUCCEEDED(hr))
			SetDebugObjectName(*pResult, "DirectXTK:PostEffectPixelShaders");

		return hr;
	});
}

// Templated base class provides functionality common to all the built-in effects.
template<typename Traits>
class PostProcessingEffectBase : public AlignedNew<XMVECTOR>
{
public:
	// Constructor.
	PostProcessingEffectBase(_In_ ID3D11Device* device)
		: dirtyFlags(INT_MAX),
		m_ConstantBuffer(device),
		m_DeviceResources(s_DeviceResourcesPool.DemandCreate(device)),
		{
			ZeroMemory(&constants, sizeof(constants));
			SetInputViewport(float2(0, 0), float2(1, 1));
		}

	typename Traits::ConstantBufferType constants;
	int dirtyFlags;
	// Helper sets our shaders and constant buffers onto the D3D device.
	void Render(_In_ ID3D11DeviceContext* deviceContext, int pass)
	{
		// Set shaders.
		auto vertexShader = m_DeviceResources->GetVertexShader();
		auto pixelShader = m_DeviceResources->GetPixelShader(pass);

		deviceContext->VSSetShader(vertexShader, nullptr, 0);
		deviceContext->PSSetShader(pixelShader, nullptr, 0);

		auto& vsCbuffer = m_DeviceResources->GetVSCbuffer();
		// Make sure the constant buffer is up to date.
		if (dirtyFlags & EffectDirtyFlags::VSConstantBuffer)
		{
			vsCbuffer.SetData(deviceContext, inputViewport);

			dirtyFlags &= ~EffectDirtyFlags::VSConstantBuffer;
		}
		// Make sure the constant buffer is up to date.
		if (dirtyFlags & EffectDirtyFlags::PSConstantBuffer)
		{
			m_ConstantBuffer.SetData(deviceContext, constants);

			dirtyFlags &= ~EffectDirtyFlags::PSConstantBuffer;
		}
		// Set the constant buffer.
		ID3D11Buffer* psbuffer = m_ConstantBuffer.GetBuffer();
		ID3D11Buffer* vsbuffer = vsCbuffer.GetBuffer();
		deviceContext->VSSetConstantBuffers(0, 1, &vsbuffer);
		deviceContext->PSSetConstantBuffers(0, 1, &psbuffer);
		m_DeviceResources->DrawQuad(deviceContext);
	}

	void SetInputViewport(float2 leftTop, float2 size)
	{
		inputViewport.uv_base = leftTop;
		inputViewport.uv_range = size;
		dirtyFlags |= EffectDirtyFlags::VSConstantBuffer;
	}
	// Helper returns the default texture.
	ID3D11ShaderResourceView* GetDefaultTexture() { return mDeviceResources->GetDefaultTexture(); }

protected:
	// Static arrays hold all the precompiled shader permutations.
	static const ShaderBytecode PixelShaderBytecode[Traits::PixelShaderCount];

	static const int PixelShaderIndices[Traits::PassCount];


private:
	// Fields.
	QuadVertexShaderConstant			inputViewport;

	// D3D constant buffer holds a copy of the same data as the public 'constants' field.
	ConstantBuffer<typename Traits::ConstantBufferType> m_ConstantBuffer;

	// Only one of these helpers is allocated per D3D device, even if there are multiple effect instances.
	class DeviceResources : public PostEffectDeviceResources
	{
	public:
		DeviceResources(_In_ ID3D11Device* device)
			: PostEffectDeviceResources(device)
		{ }

		// Gets or lazily creates the specified pixel shader permutation.
		ID3D11PixelShader* GetPixelShader(int pass)
		{
			int shaderIndex = PixelShaderIndices[pass];

			return DemandCreatePixelShader(m_PixelShaders[shaderIndex], PixelShaderBytecode[shaderIndex]);
		}

	private:
		Microsoft::WRL::ComPtr<ID3D11PixelShader>  m_PixelShaders[Traits::PixelShaderCount];
	};

	// Per-device resources.
	std::shared_ptr<DeviceResources> m_DeviceResources;

	static SharedResourcePool<ID3D11Device*, DeviceResources> s_DeviceResourcesPool;
};


namespace
{
	static const uint MaxSampleCount = 15;
	struct BlurCBufferType
	{
		float2 sampleOffsets[MaxSampleCount];
		float4 sampleWeights[MaxSampleCount];
		uint   sampleCount;
	};
}
struct BlurEffectTraits
{
	typedef BlurCBufferType ConstantBufferType;
	static const int PixelShaderCount = 1;
	static const int PassCount = 2;
};

// Base class for all 'separtable' blur kernal class
class BlurEffect : public PostProcessingEffectBase<BlurEffectTraits>
{
public:
	virtual void SetupSampleOffsetsWeights(uint kernalRadius, bool horizental) = 0;
};