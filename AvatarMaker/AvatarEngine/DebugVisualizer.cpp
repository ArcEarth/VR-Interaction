#include "stdafx.h"
#include "DebugVisualizer.h"

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