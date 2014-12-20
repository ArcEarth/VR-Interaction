#pragma once
#include <PrimitiveBatch.h>
#include <Effects.h>
#include <VertexTypes.h>
#include <CommonStates.h>
#include <wrl\client.h>
#include <GeometricPrimitive.h>
#include "DirectXMathExtend.h"
//#include "DirectXHelper.h"

namespace DirectX
{
	class GeometricPrimitiveDrawer
	{
	public:
		GeometricPrimitiveDrawer();
		void Initialize(ID3D11DeviceContext *pContext);

		void SetEffect(IEffect* pEffect);
		void XM_CALLCONV DrawCylinder(ID3D11DeviceContext *pContext, FXMVECTOR P1, FXMVECTOR P2, float radius, FXMVECTOR Color);
		void XM_CALLCONV DrawCylinder(ID3D11DeviceContext *pContext, FXMVECTOR Position, FXMVECTOR YDirection, float height, float radius, FXMVECTOR Color);
		void XM_CALLCONV DrawSphere(ID3D11DeviceContext *pContext, FXMVECTOR Position, float radius, FXMVECTOR Color);
		void XM_CALLCONV DrawCube(ID3D11DeviceContext *pContext, FXMVECTOR Position, FXMVECTOR HalfExtend, FXMVECTOR Orientation, GXMVECTOR Color);
		void XM_CALLCONV DrawCone(ID3D11DeviceContext *pContext, FXMVECTOR Position, FXMVECTOR YDirection, float height, float radius, FXMVECTOR Color);

		void XM_CALLCONV UpdateViewMatrix(DirectX::FXMMATRIX view)
		{
			XMStoreFloat4x4(&ViewMatrix, view);
		}
		void XM_CALLCONV UpdateProjectionMatrix(DirectX::FXMMATRIX projection)
		{
			XMStoreFloat4x4(&ProjectionMatrix, projection);
		}

	protected:
		Matrix4x4	ViewMatrix;
		Matrix4x4	ProjectionMatrix;
		std::unique_ptr<DirectX::GeometricPrimitive> m_pCylinder;
		std::unique_ptr<DirectX::GeometricPrimitive> m_pSphere;
		std::unique_ptr<DirectX::GeometricPrimitive> m_pCube;
		std::unique_ptr<DirectX::GeometricPrimitive> m_pCone;
	};

	class DebugVisualizer
	{
	public:

		typedef VertexPositionColor VertexType;
		DebugVisualizer();

		void Initialize(ID3D11DeviceContext *pContext);

		bool Ready() const;;

		ID3D11Device* GetDevice() const;

		ID3D11DeviceContext* GetDeviceContext() const;

		void Release();;

		DebugVisualizer(ID3D11DeviceContext *pContext);

		void SetWorld(DirectX::CXMMATRIX World);

		void SetProjection(DirectX::CXMMATRIX Projection);

		void SetView(DirectX::CXMMATRIX View);

		void Begin();

		void End();

		PrimitiveBatch<VertexPositionColor>* GetBatch();

		void DrawLine(FXMVECTOR P0, FXMVECTOR P1, FXMVECTOR Color);
		void DrawLine(FXMVECTOR P0, FXMVECTOR P1, float Width, FXMVECTOR Color);

		// Extended Line 
		void DrawCylinder(FXMVECTOR P0, FXMVECTOR P1, float Radius, FXMVECTOR Color);

		void DrawTriangle(FXMVECTOR P0, FXMVECTOR P1, FXMVECTOR P2, GXMVECTOR Color);

		void DrawQuad(FXMVECTOR P0, FXMVECTOR P1, FXMVECTOR P2, GXMVECTOR P3, CXMVECTOR Color);

		void DrawSphere(FXMVECTOR Center,float Radius,FXMVECTOR Color);
		void DrawSphere(FXMVECTOR Sphere,FXMVECTOR Color);

	protected:

	private:

		::std::unique_ptr<PrimitiveBatch<VertexPositionColor>> m_pDirectXBatch;
		::std::unique_ptr<CommonStates>	m_pStates;
		::std::unique_ptr<BasicEffect> m_pEffect;
		::Microsoft::WRL::ComPtr<ID3D11InputLayout> m_pInputLayout;
		::Microsoft::WRL::ComPtr<ID3D11DeviceContext> m_pContext;
		::Microsoft::WRL::ComPtr<ID3D11Device> m_pDevice;
	};

	__PURE_APPDOMAIN_GLOBAL extern DebugVisualizer dxout;
}
