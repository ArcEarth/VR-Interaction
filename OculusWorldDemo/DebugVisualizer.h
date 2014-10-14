#pragma once
#include <PrimitiveBatch.h>
#include <Effects.h>
#include <VertexTypes.h>
#include <wrl\client.h>
//#include "DirectXHelper.h"

namespace DirectX
{
	class DebugVisualizer
	{
	public:

		typedef VertexPositionColor VertexType;
		DebugVisualizer()
		{}

		void Initialize(ID3D11DeviceContext *pContext)
		{
			m_pContext = pContext;
			m_pContext->GetDevice(&m_pDevice);

			m_pDirectXBatch.reset(new PrimitiveBatch<VertexPositionColor>(pContext));

			m_pEffect.reset(new BasicEffect(m_pDevice.Get()));
			m_pEffect->SetVertexColorEnabled(true);
			void const* shaderByteCode;
			size_t byteCodeLength;
			m_pEffect->GetVertexShaderBytecode(&shaderByteCode, &byteCodeLength);
			HRESULT hr = m_pDevice->CreateInputLayout(VertexPositionColor::InputElements,
				VertexPositionColor::InputElementCount,
				shaderByteCode, byteCodeLength,
				&m_pInputLayout);

			assert(SUCCEEDED(hr));
		}

		bool Ready() const
		{
			return m_pContext;
		};

		ID3D11Device* GetDevice() const
		{
			return m_pDevice.Get();
		}

		ID3D11DeviceContext* GetDeviceContext() const
		{
			return m_pContext.Get();
		}

		void Release()
		{
			m_pDirectXBatch.release();
			m_pEffect.release();
			m_pInputLayout.Reset();
			m_pContext.Reset();
			m_pDevice.Reset();
		};

		DebugVisualizer(ID3D11DeviceContext *pContext)
		{
			Initialize(pContext);
		}

		void SetWorld(DirectX::CXMMATRIX World)
		{
			m_pEffect->SetWorld(World);
		}

		void SetProjection(DirectX::CXMMATRIX Projection)
		{
			m_pEffect->SetProjection(Projection);
		}

		void SetView(DirectX::CXMMATRIX View)
		{
			m_pEffect->SetView(View);
		}

		void Begin()
		{
			m_pEffect->Apply(m_pContext.Get());
			m_pContext->IASetInputLayout(m_pInputLayout.Get());
			m_pDirectXBatch->Begin();
		}

		void End()
		{
			m_pDirectXBatch->End();
		}

		void DrawLine(FXMVECTOR P0,FXMVECTOR P1,FXMVECTOR Color)
		{
			VertexType Vertices[] = {VertexType(P0,Color),VertexType(P1,Color)};
			m_pDirectXBatch->DrawLine(Vertices[0],Vertices[1]);
		}

		void DrawTriangle(FXMVECTOR P0,FXMVECTOR P1,FXMVECTOR P2,GXMVECTOR Color)
		{
			VertexType Vertices[] = {VertexType(P0,Color),VertexType(P1,Color),VertexType(P2,Color)};
			m_pDirectXBatch->DrawTriangle(Vertices[0],Vertices[1],Vertices[2]);
		}

		void DrawQuad(FXMVECTOR P0,FXMVECTOR P1,FXMVECTOR P2,GXMVECTOR P3,CXMVECTOR Color)
		{
			VertexType Vertices[] = {VertexType(P0,Color),VertexType(P1,Color),VertexType(P2,Color),VertexType(P3,Color)};
			m_pDirectXBatch->DrawQuad(Vertices[0],Vertices[1],Vertices[2],Vertices[3]);
		}

		void DrawSphere(FXMVECTOR Center,float Radius,FXMVECTOR Color);
		void DrawSphere(FXMVECTOR Sphere,FXMVECTOR Color);

	protected:

	private:

		::std::unique_ptr<PrimitiveBatch<VertexPositionColor>> m_pDirectXBatch;
		::std::unique_ptr<BasicEffect> m_pEffect;
		::Microsoft::WRL::ComPtr<ID3D11InputLayout> m_pInputLayout;
		::Microsoft::WRL::ComPtr<ID3D11DeviceContext> m_pContext;
		::Microsoft::WRL::ComPtr<ID3D11Device> m_pDevice;
	};

	__PURE_APPDOMAIN_GLOBAL extern DebugVisualizer dxout;
}
