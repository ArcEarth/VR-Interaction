#pragma once
#if defined(_XBOX_ONE) && defined(_TITLE)
#include <d3d11_x.h>
#else
#include <d3d11_1.h>
#endif
#include <wrl\client.h>
#include <Effects.h>
#include "DirectXHelper.h"
#include <ppltasks.h>
#include <memory>

namespace DirectX
{
	class EffectBank
	{
		template <typename _TVertex>
		std::shared_ptr<IEffect> CreateEffect()
		{
			return nullptr;
		}
	};

	class IEffectMaterial
	{
		virtual void SetDiffuseTexture(ID3D11ShaderResourceView* pTexture) = 0;
		virtual void SetNormalTexture(ID3D11ShaderResourceView* pTexture) = 0;
		virtual void SetSpecularTexture(ID3D11ShaderResourceView* pTexture) = 0;
	};

	class IEffectShadowMap abstract
	{
		virtual void SetShadowMap(int shadowId, ID3D11ShaderResourceView* pTexture) = 0;
		virtual void XM_CALLCONV SetShadowColor(int shadowId, FXMVECTOR color) = 0;
	};

	class IEffectLightMap
	{

	};

	//ComPtr<ID3D11InputLayout> CreateInputLayout(ID3D11Device* pDevice, IEffect* pEffect)
	//{
	//	void const* shaderByteCode;
	//	size_t byteCodeLength;
	//	pEffect->GetVertexShaderBytecode(&shaderByteCode, &byteCodeLength);
	//	return DirectX::CreateInputLayout(pDevice, shaderByteCode, byteCodeLength);
	//}

	// An base class for customized shader effect, will handel the loading of shaders
	class IShaderEffect
	{
	public:
		virtual ~IShaderEffect() { }
		virtual ID3D11InputLayout*		GetInputLayout() const = 0;
		virtual ID3D11VertexShader*		GetVertexShader() const = 0;
		virtual ID3D11GeometryShader*	GetGeometryShader() const = 0;
		virtual ID3D11PixelShader*		GetPixelShader() const = 0;
	};

	class ShaderEffect
		: public IEffect , public IShaderEffect
	{
	public:
		ShaderEffect() {}

		template <typename _TVertex>
		ShaderEffect(_In_ ID3D11Device *pDevice, _In_ const char *VertexShaderFile, _In_opt_ const char *GeometryShaderFile , const char *PixelShaderFile)
		{
			typedef typename _TVertex VertexType;
			std::string fileData = DirectX::ReadFileToString(VertexShaderFile);
			m_VertexShaderByteCode = fileData;
			m_VertexShaderByteCode.shrink_to_fit();
			//Create the VertexShader
			DirectX::ThrowIfFailed(
				pDevice->CreateVertexShader(
				fileData.c_str(),
				fileData.size(),
				nullptr,
				m_pVertexShader.GetAddressOf()
				));
			//Create the InputLayout
			m_pInputLayout = CreateInputLayout<VertexType>(pDevice, fileData.c_str(), fileData.size());

			//Create the PixelShader
			fileData = DirectX::ReadFileToString(PixelShaderFile);
			DirectX::ThrowIfFailed(
				pDevice->CreatePixelShader(
				fileData.c_str(),
				fileData.size(),
				nullptr,
				m_pPixelShader.GetAddressOf()
				));

			//Create the GeometryShader
			if (GeometryShaderFile != nullptr) {
				fileData = DirectX::ReadFileToString(GeometryShaderFile);
				DirectX::ThrowIfFailed(
					pDevice->CreateGeometryShader(
					fileData.c_str(),
					fileData.size(),
					nullptr,
					m_pGeometryShader.GetAddressOf()
					));
			}
		}

		template <size_t VertexShaderBytecodeLength, size_t PixelShaderBytecodeLength>
		ShaderEffect(_In_ ID3D11Device *pDevice, _In_ const BYTE (&VertexShaderBytecode const)[VertexShaderBytecodeLength], _In_ const BYTE(&PixelShaderBytecode const)[PixelShaderBytecodeLength])
		{
		}

		~ShaderEffect()
		{
		}

		ID3D11InputLayout* GetInputLayout() const
		{
			return m_pInputLayout.Get();
		}
		ID3D11VertexShader* GetVertexShader() const
		{
			return m_pVertexShader.Get();
		}
		ID3D11GeometryShader* GetGeometryShader() const
		{
			return m_pGeometryShader.Get();
		}
		ID3D11PixelShader* GetPixelShader() const
		{
			return m_pPixelShader.Get();
		}

		virtual void Apply(_In_ ID3D11DeviceContext* pDeviceContext)
		{
			pDeviceContext->IASetInputLayout(m_pInputLayout.Get());
			pDeviceContext->VSSetShader(m_pVertexShader.Get(), nullptr, 0);
			pDeviceContext->GSSetShader(m_pGeometryShader.Get(), nullptr, 0);
			pDeviceContext->PSSetShader(m_pPixelShader.Get(), nullptr, 0);
		}

		virtual void GetVertexShaderBytecode(_Out_ void const** pShaderByteCode, _Out_ size_t* pByteCodeLength)
		{
			*pShaderByteCode = m_VertexShaderByteCode.c_str();
			*pByteCodeLength = m_VertexShaderByteCode.size();
		}

	protected:
		std::string						m_VertexShaderByteCode;
		ComPtr<ID3D11VertexShader>		m_pVertexShader;
		ComPtr<ID3D11GeometryShader>	m_pGeometryShader;
		ComPtr<ID3D11PixelShader>		m_pPixelShader;
		ComPtr<ID3D11InputLayout>		m_pInputLayout;
	};

	template <typename _TVSConstant, typename _TPSConstant, typename _TGSConstant>
	class CommonShaderEffect
	{

	};
}
