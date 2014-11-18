#include "Carmera.h"
#include "DirectXHelper.h"

//using namespace DirectX;

//PerspectiveCamera::PerspectiveCamera( ID3D11Device *pDevice , UINT CameraBufferSlot , const float FovAngleY , const float AspectHbyW , const float NearZ /*= 0.1f */, const float FarZ /*= 1000.0f*/ ) : FOV(FovAngleY) , Aspect(AspectHbyW) , Near(NearZ) , Far(FarZ)
//{
//	m_Slot = CameraBufferSlot;
//	m_pGPUBuffer = DirectX::CreateConstantBuffer<CameraBuffer>(pDevice);
//}
//
//DirectX::XMMATRIX PerspectiveCamera::GetViewMatrix() const
//{
//	DirectX::Vector3 focus(0.0f,0.0f,1.0f);
//	focus.Rotate(Orientation);
//	focus += Position;
//	DirectX::Vector3 up(0.0f,1.0f,0.0f);
//	up.Rotate(Orientation);
////	return XMMatrixLookAtLH(Position,focus,up);
//	return XMMatrixLookAtRH(Position,focus,up);
//}
//
//DirectX::XMVECTOR PerspectiveCamera::GetPosition() const
//{
//	return Position;
//}
//
//DirectX::XMMATRIX PerspectiveCamera::GetProjectionMatrix() const
//{
////	return DirectX::XMMatrixPerspectiveFovLH(FOV,Aspect,Near,Far);
//	return DirectX::XMMatrixPerspectiveFovRH(FOV,Aspect,Near,Far);
//}
//
//ID3D11Buffer*	PerspectiveCamera::GetBuffer() const
//{
//	return m_pGPUBuffer;
//}
//
//ID3D11Buffer*	PerspectiveCamera::UpdateBuffer(ID3D11DeviceContext* pDeviceContext)
//{
//	if (!m_DirtyFlag)
//		return m_pGPUBuffer;
//
//	if (m_DirtyFlag & DirtyFlags::ViewMatrix)
//	{
//		DirectX::Vector3 focus(0.0f,0.0f,1.0f);
//		focus.Rotate(Orientation);
//		focus += Position;
//		DirectX::Vector3 up(0.0f,1.0f,0.0f);
//		up.Rotate(Orientation);
//		m_Buffer.CameraPosition = (DirectX::XMFLOAT4)Position;
//		m_Buffer.CameraFocus = (DirectX::XMFLOAT4)focus;
//		XMMATRIX ViewMatrix = XMMatrixLookAtRH(Position,focus,up);
//		XMStoreFloat4x4(&m_Buffer.ViewMatrix , ViewMatrix);
//		m_DirtyFlag &= ~DirtyFlags::ViewMatrix;
//	}
//
//	if (m_DirtyFlag & DirtyFlags::ProjectionMatrix)
//	{
//		XMMATRIX ProjectionMatrix = DirectX::XMMatrixPerspectiveFovRH(FOV,Aspect,Near,Far);
//		XMStoreFloat4x4(&m_Buffer.ProjectionMatrix , ProjectionMatrix);
//		m_DirtyFlag &= ~DirtyFlags::ProjectionMatrix;
//	}
//
//	DirectX::XMMATRIX ViewProjectionMatrix = XMMatrixMultiplyTranspose(XMLoadFloat4x4(&m_Buffer.ViewMatrix),XMLoadFloat4x4(&m_Buffer.ProjectionMatrix));
//	XMStoreFloat4x4(&(m_Buffer.ViewProjectionMatrix),ViewProjectionMatrix);
//
//
//	////Maybe the order is wrong
//	//DirectX::XMMATRIX ViewProjectionMatrix = XMMatrixMultiplyTranspose(XMMatrixLookAtRH(Position,focus,up),DirectX::XMMatrixPerspectiveFovRH(FOV,Aspect,Near,Far));
//	//XMStoreFloat4x4(&(m_Buffer.ViewProjectionMatrix),ViewProjectionMatrix);
//
//
//	DirectX::SetBufferData(m_pGPUBuffer,pDeviceContext,m_Buffer);
//	return m_pGPUBuffer;
//}
//
//
//
//void PerspectiveCamera::Render( ID3D11DeviceContext *pContext )
//{
//	// update the content of the CameraBuffer
//	DirectX::Vector3 focus(0.0f,0.0f,1.0f);
//	focus.Rotate(Orientation);
//	focus += Position;
//	DirectX::Vector3 up(0.0f,1.0f,0.0f);
//	up.Rotate(Orientation);
//	m_Buffer.CameraPosition = (DirectX::XMFLOAT4)Position;
//	m_Buffer.CameraFocus = (DirectX::XMFLOAT4)focus;
//	//		m_Buffer.CameraUpDirection = (XMFLOAT4)up;
//
//	//Maybe the order is wrong
//	DirectX::XMMATRIX ViewProjectionMatrix = XMMatrixMultiplyTranspose(XMMatrixLookAtRH(Position,focus,up),DirectX::XMMatrixPerspectiveFovRH(FOV,Aspect,Near,Far));
//	XMStoreFloat4x4(&(m_Buffer.ViewProjectionMatrix),ViewProjectionMatrix);
//
//	// update them to GPU
//	DirectX::SetBufferData(m_pGPUBuffer,pContext,m_Buffer);
//	pContext->VSSetConstantBuffers(m_Slot,1,&m_pGPUBuffer);
//}
//
//PerspectiveCamera::~PerspectiveCamera()
//{
//	DirectX::SafeRelease(m_pGPUBuffer);
//}
//
//void PerspectiveCamera::Move(DirectX::FXMVECTOR DisplacementVector)
//{
//	m_DirtyFlag |= ViewMatrix;
//	XMVECTOR Q = Orientation;
//	XMVECTOR V = XMVector3Rotate(DisplacementVector,Q);
//	Position += (Vector3)V;
//}
//
//void PerspectiveCamera::Turn(DirectX::FXMVECTOR TurningQuternion)
//{
//	m_DirtyFlag |= ViewMatrix;
//	XMVECTOR Q = Orientation;
//	Q = XMQuaternionMultiply(TurningQuternion,Q);
//	Orientation = Q;
//}
//
