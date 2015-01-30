
#pragma once
#include "stdafx.h"
#include <wrl.h>
#include <Effects.h>
#include "DirectXHelper.h"

namespace DirectX
{
	template <typename _TVertex>
	class CustomEffect
		: public IEffect
	{
	public:
		typedef typename _TVertex VertexType;

		CustomEffect(ID3D11Device *pDevice , const char *VertexShaderFile , const char *GeometryShaderFile , const char *PixelShaderFile)
		{
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
			DirectX::ThrowIfFailed(
				pDevice->CreateInputLayout(
				VertexType::InputElements,
				VertexType::InputElementCount,
				fileData.c_str(),
				fileData.size(),
				m_pInputLayout.GetAddressOf()
				));

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
			if (GeometryShaderFile!=nullptr) {
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

		~CustomEffect(void)
		{
		}

		virtual void Apply(_In_ ID3D11DeviceContext* pDeviceContext)
		{
			pDeviceContext->IASetInputLayout(m_pInputLayout.Get());
			pDeviceContext->VSSetShader(m_pVertexShader.Get(),nullptr,0);
			pDeviceContext->GSSetShader(m_pGeometryShader.Get(),nullptr,0);
			pDeviceContext->PSSetShader(m_pPixelShader.Get(),nullptr,0);
		}

		virtual void GetVertexShaderBytecode(_Out_ void const** pShaderByteCode, _Out_ size_t* pByteCodeLength)
		{
			*pShaderByteCode = m_VertexShaderByteCode.c_str();
			*pByteCodeLength = m_VertexShaderByteCode.size();
		}


	protected:
		std::string m_VertexShaderByteCode;
		Microsoft::WRL::ComPtr<ID3D11VertexShader> m_pVertexShader;
		Microsoft::WRL::ComPtr<ID3D11GeometryShader> m_pGeometryShader;
		Microsoft::WRL::ComPtr<ID3D11PixelShader> m_pPixelShader;
		Microsoft::WRL::ComPtr<ID3D11InputLayout> m_pInputLayout;
	};


}
