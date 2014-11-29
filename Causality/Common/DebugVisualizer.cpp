#include "DebugVisualizer.h"
#include <vector>

//using namespace DirectX;
namespace DirectX{
	namespace Internal{
		// 34 Vertecies
		static const XMVECTORF32 SphereVertices[] = {
			{0.0000f,1.0000f,0.0000f,0.0000f},
			{0.4600f,0.8880f,0.0000f,0.0000f},
			{-0.2300f,0.8880f,-0.3980f,0.0000f},
			{0.8160f,0.5770f,0.0000f,0.0000f},
			{0.2890f,0.8160f,-0.5000f,0.0000f},
			{-0.4080f,0.5770f,-0.7070f,0.0000f},
			{0.9910f,0.1370f,0.0000f,0.0000f},
			{0.7450f,0.2520f,-0.6170f,0.0000f},
			{0.1620f,0.2520f,-0.9540f,0.0000f},
			{-0.4950f,0.1370f,-0.8580f,0.0000f},
			{0.9430f,-0.3330f,0.0000f,0.0000f},
			{0.7610f,-0.5130f,-0.3980f,0.0000f},
			{0.4080f,-0.5770f,-0.7070f,0.0000f},
			{-0.0360f,-0.5130f,-0.8580f,0.0000f},
			{-0.4710f,-0.3330f,-0.8160f,0.0000f},
			{-0.2300f,0.8880f,0.3980f,0.0000f},
			{-0.5770f,0.8160f,0.0000f,0.0000f},
			{-0.4080f,0.5770f,0.7070f,0.0000f},
			{-0.9070f,0.2520f,-0.3370f,0.0000f},
			{-0.9070f,0.2520f,0.3370f,0.0000f},
			{-0.4950f,0.1370f,0.8580f,0.0000f},
			{-0.7250f,-0.5130f,-0.4600f,0.0000f},
			{-0.8160f,-0.5770f,0.0000f,0.0000f},
			{-0.7250f,-0.5130f,0.4600f,0.0000f},
			{-0.4710f,-0.3330f,0.8160f,0.0000f},
			{0.2890f,0.8160f,0.5000f,0.0000f},
			{0.1620f,0.2520f,0.9540f,0.0000f},
			{0.7450f,0.2520f,0.6170f,0.0000f},
			{-0.0360f,-0.5130f,0.8580f,0.0000f},
			{0.4080f,-0.5770f,0.7070f,0.0000f},
			{0.7610f,-0.5130f,0.3980f,0.0000f},
			{0.5770f,-0.8160f,0.0000f,0.0000f},
			{-0.0650f,-0.9390f,0.3370f,0.0000f},
			{-0.0650f,-0.9390f,-0.3370f,0.0000f},
		};

		// 64 Facets
		static const uint16_t SphereIndics[] = {
			0,1,2,
			1,3,4,
			1,4,2,
			2,4,5,
			3,6,7,
			3,7,4,
			4,7,8,
			4,8,5,
			5,8,9,
			6,10,11,
			6,11,7,
			7,11,12,
			7,12,8,
			8,12,13,
			8,13,9,
			9,13,14,
			0,2,15,
			2,5,16,
			2,16,15,
			15,16,17,
			5,9,18,
			5,18,16,
			16,18,19,
			16,19,17,
			17,19,20,
			9,14,21,
			9,21,18,
			18,21,22,
			18,22,19,
			19,22,23,
			19,23,20,
			20,23,24,
			0,15,1,
			15,17,25,
			15,25,1,
			1,25,3,
			17,20,26,
			17,26,25,
			25,26,27,
			25,27,3,
			3,27,6,
			20,24,28,
			20,28,26,
			26,28,29,
			26,29,27,
			27,29,30,
			27,30,6,
			6,30,10,
			10,30,11,
			30,29,31,
			30,31,11,
			11,31,12,
			29,28,32,
			29,32,31,
			31,32,33,
			31,33,12,
			12,33,13,
			28,24,23,
			28,23,32,
			32,23,22,
			32,22,33,
			33,22,21,
			33,21,13,
			13,21,14,
		};

		std::vector<VertexPositionColor> CylinderVertices;
		std::vector<uint16_t> CylinderIndices;
	}

	DebugVisualizer::DebugVisualizer()
		{}

	void DebugVisualizer::Initialize(ID3D11DeviceContext * pContext)
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

	bool DebugVisualizer::Ready() const
	{
		return m_pContext;
	}

	ID3D11Device * DebugVisualizer::GetDevice() const
	{
		return m_pDevice.Get();
	}

	ID3D11DeviceContext * DebugVisualizer::GetDeviceContext() const
	{
		return m_pContext.Get();
	}

	void DebugVisualizer::Release()
	{
		m_pDirectXBatch.release();
		m_pEffect.release();
		m_pInputLayout.Reset();
		m_pContext.Reset();
		m_pDevice.Reset();
	}

	DebugVisualizer::DebugVisualizer(ID3D11DeviceContext * pContext)
	{
		Initialize(pContext);
	}

	void DebugVisualizer::SetWorld(DirectX::CXMMATRIX World)
	{
		m_pEffect->SetWorld(World);
	}

	void DebugVisualizer::SetProjection(DirectX::CXMMATRIX Projection)
	{
		m_pEffect->SetProjection(Projection);
	}

	void DebugVisualizer::SetView(DirectX::CXMMATRIX View)
	{
		m_pEffect->SetView(View);
	}

	void DebugVisualizer::Begin()
	{
		m_pEffect->Apply(m_pContext.Get());
		m_pContext->IASetInputLayout(m_pInputLayout.Get());
		m_pDirectXBatch->Begin();
	}

	void DebugVisualizer::End()
	{
		m_pDirectXBatch->End();
	}

	void DebugVisualizer::DrawLine(FXMVECTOR P0, FXMVECTOR P1, FXMVECTOR Color)
	{
		VertexType Vertices [] = { VertexType(P0,Color),VertexType(P1,Color) };
		m_pDirectXBatch->DrawLine(Vertices[0], Vertices[1]);
	}

	void DebugVisualizer::DrawLine(FXMVECTOR P0, FXMVECTOR P1, float Width, FXMVECTOR Color)
	{
		DrawLine(P0, P1, Color);
	}

	void DebugVisualizer::DrawCylinder(FXMVECTOR P0, FXMVECTOR P1, float Radius, FXMVECTOR Color)
	{
		VertexType Vertices [] = { VertexType(P0,Color),VertexType(P1,Color) };
		m_pDirectXBatch->DrawLine(Vertices[0], Vertices[1]);
	}

		void DebugVisualizer::DrawTriangle(FXMVECTOR P0, FXMVECTOR P1, FXMVECTOR P2, GXMVECTOR Color)
	{
		VertexType Vertices [] = { VertexType(P0,Color),VertexType(P1,Color),VertexType(P2,Color) };
		m_pDirectXBatch->DrawTriangle(Vertices[0], Vertices[1], Vertices[2]);
	}

	void DebugVisualizer::DrawQuad(FXMVECTOR P0, FXMVECTOR P1, FXMVECTOR P2, GXMVECTOR P3, CXMVECTOR Color)
	{
		VertexType Vertices [] = { VertexType(P0,Color),VertexType(P1,Color),VertexType(P2,Color),VertexType(P3,Color) };
		m_pDirectXBatch->DrawQuad(Vertices[0], Vertices[1], Vertices[2], Vertices[3]);
	}

		void DebugVisualizer::DrawSphere(FXMVECTOR Center,float Radius,FXMVECTOR Color)
	{
		VertexPositionColor Vertices[34];
		XMVECTOR vRadius = XMVectorReplicate(Radius);

		for (int i = 0; i < 34; i++)
		{
			XMVECTOR vtr = Internal::SphereVertices[i] * vRadius + Center;
			XMStoreFloat3(&Vertices[i].position,vtr);
			XMStoreFloat4(&Vertices[i].color,Color);
		}
		m_pDirectXBatch->DrawIndexed(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST,Internal::SphereIndics,64*3,Vertices,34);
	}

	void DebugVisualizer::DrawSphere(FXMVECTOR Sphere,FXMVECTOR Color)
	{
		VertexPositionColor Vertices[34];
		XMVECTOR vRadius = XMVectorSwizzle<3,3,3,3>(Sphere);

		for (int i = 0; i < 34; i++)
		{
			XMVECTOR vtr = Internal::SphereVertices[i] * vRadius + Sphere;
			XMStoreFloat3(&Vertices[i].position,vtr);
			XMStoreFloat4(&Vertices[i].color,Color);
		}
		m_pDirectXBatch->DrawIndexed(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST,Internal::SphereIndics,64*3,Vertices,34);
	}

	DebugVisualizer dxout;
}