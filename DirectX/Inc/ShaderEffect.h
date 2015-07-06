#pragma once
#if defined(_XBOX_ONE) && defined(_TITLE)
#include <d3d11_x.h>
#else
#include <d3d11_1.h>
#endif
#include <wrl\client.h>
#include <Effects.h>
#include <ppltasks.h>
#include <memory>

namespace DirectX
{
	class IEffectPhongMaterial
	{
	public:
		// Material settings.
		virtual void XM_CALLCONV SetDiffuseColor(FXMVECTOR value) = 0;
		virtual void XM_CALLCONV SetEmissiveColor(FXMVECTOR value) = 0;
		virtual void XM_CALLCONV SetSpecularColor(FXMVECTOR value) = 0;
		virtual void __cdecl SetSpecularPower(float value) = 0;
		virtual void __cdecl DisableSpecular() = 0;
		virtual void __cdecl SetAlpha(float value) = 0;

		virtual void SetDiffuseMap(ID3D11ShaderResourceView* pTexture) = 0;
		virtual void SetNormalMap(ID3D11ShaderResourceView* pTexture) = 0;
		virtual void SetSpecularMap(ID3D11ShaderResourceView* pTexture) = 0;
	};

	class IEffectLightsShadow abstract : public IEffectLights
	{
	public:
		// Lighting and per pixel lighting is always enabled for Shadowed Light Effects
		virtual void __cdecl SetLightingEnabled(bool value) {};
		virtual void __cdecl SetPerPixelLighting(bool value) {};
		virtual void XM_CALLCONV SetAmbientLightColor(FXMVECTOR value) {};
		virtual void XM_CALLCONV SetLightDirection(int whichLight, FXMVECTOR value) {};
		virtual void XM_CALLCONV SetLightDiffuseColor(int whichLight, FXMVECTOR value) {};
		virtual void XM_CALLCONV SetLightSpecularColor(int whichLight, FXMVECTOR value) {};

		// Methods need for IEffectLightsShadow
		virtual void __cdecl SetLightEnabled(int whichLight, bool value) override = 0;
		virtual void __cdecl SetLightShadowMapBias(int whichLight, float bias) = 0;
		virtual void __cdecl SetLightShadowMap(int whichLight, ID3D11ShaderResourceView* pTexture) = 0;
		virtual void XM_CALLCONV SetLightView(int whichLight, FXMMATRIX value) = 0;
		virtual void XM_CALLCONV SetLightProjection(int whichLight, FXMMATRIX value) = 0;
	};

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
		ShaderEffect(_In_ ID3D11Device *pDevice, _In_ const BYTE (&VertexShaderBytecode)[VertexShaderBytecodeLength], _In_ const BYTE(&PixelShaderBytecode)[PixelShaderBytecodeLength])
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

		Microsoft::WRL::ComPtr<ID3D11VertexShader>		m_pVertexShader;
		Microsoft::WRL::ComPtr<ID3D11GeometryShader>	m_pGeometryShader;
		Microsoft::WRL::ComPtr<ID3D11PixelShader>		m_pPixelShader;
		Microsoft::WRL::ComPtr<ID3D11InputLayout>		m_pInputLayout;
	};

	template <typename _TVSConstant, typename _TPSConstant, typename _TGSConstant>
	class CommonShaderEffect
	{

	};
}
