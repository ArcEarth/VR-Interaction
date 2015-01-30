#pragma once
#include "stdafx.h"
#include <wrl\client.h>
#include <VertexTypes.h>
#include "Textures.h"
#include <VertexTypes.h>
#include "ConstantBuffer.h"
#include <CommonStates.h>

namespace DirectX
{
	class PostProcesser
	{
	public:
		PostProcesser(void)
			: m_Initialized(false)
		{}

		PostProcesser(ID3D11Device* pDevice)
			: m_Initialized(false)
		{
			Initialize(pDevice);
		}

		~PostProcesser(void)
		{}

		/// <summary>
		/// Presents the source texture to the render target with in specified device context.
		/// Equal to perform a STRECH copy from a texture to another texture.
		/// </summary>
		/// <param name="pDeviceContext">The specified device context.</param>
		/// <param name="pSource">The pointer to source texture.</param>
		/// <param name="pTraget">The pointer to render target.</param>
		void Present(ID3D11DeviceContext *pDeviceContext ,const Texture2D *pSource, DirectX::RenderTargetTexture2D* pTarget)
		{
			pTarget->SetAsRenderTarget(pDeviceContext);
			ID3D11SamplerState* pSampler;
			if (pTarget->Width() == pSource->Width() && pTarget->Height() == pSource->Height())
			{
				pSampler = m_pCommanStates->PointClamp();
			} else
			{
				pSampler = m_pCommanStates->LinearClamp();
			}
			pDeviceContext->PSSetSamplers(0,1,&pSampler);
		}


		//void Present(ID3D11DeviceContext *pDeviceContext , ID3D11ShaderResourceView *pSource, DirectX::RenderTargetTexture2D* pTraget , ID3D11SamplerState* pSampler = nullptr)
		//{
		//	pTraget->SetAsRenderTarget(pDeviceContext);
		//	pDeviceContext->PSSetShader(m_pPresentPixelShader.Get(),nullptr,0);
		//	pDeviceContext->PSSetShaderResources(0,1,&pSource);
		//	pDeviceContext->PSSetSamplers(0,1,&pSampler);
		//	Draw(pDeviceContext);
		//}

		void Blur(ID3D11DeviceContext *pDeviceContext , ID3D11ShaderResourceView *pSource , DirectX::RenderTargetTexture2D* pTraget)
		{
			ID3D11SamplerState* pSampler = m_pCommanStates->LinearClamp();
			BlurConstants HorizenGaussBlurConstant , VecticalGaussBlurConstant;

			pDeviceContext->PSSetShader(m_pBlurPixelShader.Get(),nullptr,0);
			pDeviceContext->PSSetSamplers(0,1,&pSampler);

			m_pBlurMediumTextures[0]->SetAsRenderTarget(pDeviceContext);
			pDeviceContext->PSSetShaderResources(0,1,&pSource);
			m_pBlurPSBlurConstantBuffer->SetData(pDeviceContext,HorizenGaussBlurConstant);
			auto constant = m_pBlurPSBlurConstantBuffer->GetBuffer();
			pDeviceContext->PSSetConstantBuffers(0,1,&constant);
			Draw(pDeviceContext);

			m_pBlurMediumTextures[1]->SetAsRenderTarget(pDeviceContext);
			auto source = m_pBlurMediumTextures[0]->ShaderResourceView();
			pDeviceContext->PSSetShaderResources(0,1,&source);
			m_pBlurPSBlurConstantBuffer->SetData(pDeviceContext,VecticalGaussBlurConstant);
			constant = m_pBlurPSBlurConstantBuffer->GetBuffer();
			pDeviceContext->PSSetConstantBuffers(0,1,&constant);
			Draw(pDeviceContext);

			pTraget->SetAsRenderTarget(pDeviceContext);
			pDeviceContext->PSSetShader(m_pBlurPixelShader.Get(),nullptr,0);
			pDeviceContext->PSSetShaderResources(0,1,&pSource);
			pDeviceContext->PSSetSamplers(0,1,&pSampler);
			Draw(pDeviceContext);
		}
		

		bool IsInitialized() const
		{
			return m_Initialized;
		}

		Concurrency::task<void> Initialize(ID3D11Device* pDevice)
		{
			auto loadVSTask = DirectX::ReadDataAsync(L"VS_Quad.cso");

			auto createVSTask = loadVSTask.then([this , pDevice](std::string fileData){
				DirectX::ThrowIfFailed(
					pDevice->CreateVertexShader(fileData.c_str(),fileData.length(),nullptr,&m_pVertexShader)
				);
			});

			auto loadPresentPSTask = DirectX::ReadDataAsync(L"PS_Post_Present.cso");
			auto createPresentPSTask = loadPresentPSTask.then([this , pDevice](std::string fileData){
				DirectX::ThrowIfFailed(
					pDevice->CreatePixelShader(fileData.c_str(),fileData.length(),nullptr,&m_pPresentPixelShader)
				);
			});

			auto loadBlurPSTask = DirectX::ReadDataAsync(L"PS_Post_Blur.cso");
			auto createBlurPSTask = loadBlurPSTask.then([this , pDevice](std::string fileData){
				DirectX::ThrowIfFailed(
					pDevice->CreatePixelShader(fileData.c_str(),fileData.length(),nullptr,&m_pBlurPixelShader)
				);
			});

			auto loadComposePSTask = DirectX::ReadDataAsync(L"PS_Post_Compose.cso");
			auto createComposePSTask = loadComposePSTask.then([this , pDevice](std::string fileData){
				DirectX::ThrowIfFailed(
					pDevice->CreatePixelShader(fileData.c_str(),fileData.length(),nullptr,&m_pComposePixelShader)
				);
			});

			auto FinishTask = (createPresentPSTask && createVSTask && createComposePSTask && createBlurPSTask).then([this](){
				m_Initialized = true;
			});

			m_pCommanStates.reset(new CommonStates(pDevice));

			// For storage the down sampled texture and H/V blurred texture
			m_pBlurMediumTextures[0].reset(new RenderTargetTexture2D(pDevice,683,384));
			// For storage the horizon blurred texture
			m_pBlurMediumTextures[1].reset(new RenderTargetTexture2D(pDevice,683,384));

			m_pBlurPSBlurConstantBuffer.reset(new ConstantBuffer<BlurConstants>(pDevice));

			return FinishTask;
		}

	protected:
		// Call this method to draw a full screen wise quad
		void Draw(ID3D11DeviceContext *pContext)
		{
			pContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP);
			pContext->VSSetShader(m_pVertexShader.Get(), nullptr, 0);
			pContext->GSSetShader(nullptr, nullptr, 0);
			pContext->Draw(4, 0);
		};

	protected:
		static const unsigned int BLUR_PIXEL_COUNT = 5;
		struct BlurConstants
		{
			XMFLOAT2 SampleOffsets[BLUR_PIXEL_COUNT];
			XMFLOAT4 SampleWeights[BLUR_PIXEL_COUNT];
		};

		bool											m_Initialized;
		// The vertex shader generator a full screen quard and send to pixel shader to post process.
		Microsoft::WRL::ComPtr<ID3D11VertexShader>		m_pVertexShader;
		Microsoft::WRL::ComPtr<ID3D11PixelShader>		m_pPresentPixelShader;
		Microsoft::WRL::ComPtr<ID3D11PixelShader>		m_pBlurPixelShader;
		Microsoft::WRL::ComPtr<ID3D11PixelShader>		m_pComposePixelShader;

		std::unique_ptr<ConstantBuffer<BlurConstants>>	m_pBlurPSBlurConstantBuffer;
		std::unique_ptr<RenderTargetTexture2D>			m_pBlurMediumTextures[2];
		std::unique_ptr<CommonStates>					m_pCommanStates;
	};


}