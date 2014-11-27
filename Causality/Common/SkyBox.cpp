#include "SkyBox.h"
#include <DDSTextureLoader.h>

using namespace std;
namespace DirectX
{

	namespace Scene
	{

		const SkyBox::VertexType SkyBox::CubeVertices[24] =
		{
			// positive x
			VertexPositionTexture(XMFLOAT3(1.0f, -1.0f,  1.0f) , XMFLOAT2(0.0f, 1.0f)),
			VertexPositionTexture(XMFLOAT3(1.0f,  1.0f,  1.0f) , XMFLOAT2(0.0f, 0.0f)),
			VertexPositionTexture(XMFLOAT3(1.0f,  1.0f, -1.0f) , XMFLOAT2(1.0f, 0.0f)),
			VertexPositionTexture(XMFLOAT3(1.0f, -1.0f, -1.0f) , XMFLOAT2(1.0f, 1.0f)),

			// negative x
			VertexPositionTexture(XMFLOAT3(-1.0f, -1.0f, -1.0f) , XMFLOAT2(0.0f, 1.0f)),
			VertexPositionTexture(XMFLOAT3(-1.0f,  1.0f, -1.0f) , XMFLOAT2(0.0f, 0.0f)),
			VertexPositionTexture(XMFLOAT3(-1.0f,  1.0f,  1.0f) , XMFLOAT2(1.0f, 0.0f)),
			VertexPositionTexture(XMFLOAT3(-1.0f, -1.0f,  1.0f) , XMFLOAT2(1.0f, 1.0f)),

			// positive y
			VertexPositionTexture(XMFLOAT3(-1.0f,  1.0f,  1.0f) , XMFLOAT2(0.0f, 1.0f)),
			VertexPositionTexture(XMFLOAT3(-1.0f,  1.0f, -1.0f) , XMFLOAT2(0.0f, 0.0f)),
			VertexPositionTexture(XMFLOAT3(1.0f,  1.0f, -1.0f) , XMFLOAT2(1.0f, 0.0f)),
			VertexPositionTexture(XMFLOAT3(1.0f,  1.0f,  1.0f) , XMFLOAT2(1.0f, 1.0f)),

			// negative y
			VertexPositionTexture(XMFLOAT3(1.0f, -1.0f, -1.0f) , XMFLOAT2(0.0f, 1.0f)),
			VertexPositionTexture(XMFLOAT3(-1.0f, -1.0f, -1.0f) , XMFLOAT2(0.0f, 0.0f)),
			VertexPositionTexture(XMFLOAT3(-1.0f, -1.0f,  1.0f) , XMFLOAT2(1.0f, 0.0f)),
			VertexPositionTexture(XMFLOAT3(1.0f, -1.0f,  1.0f) , XMFLOAT2(1.0f, 1.0f)),

			// positive z
			VertexPositionTexture(XMFLOAT3(-1.0f, -1.0f,  1.0f) , XMFLOAT2(0.0f, 1.0f)),
			VertexPositionTexture(XMFLOAT3(-1.0f,  1.0f,  1.0f) , XMFLOAT2(0.0f, 0.0f)),
			VertexPositionTexture(XMFLOAT3(1.0f,  1.0f,  1.0f) , XMFLOAT2(1.0f, 0.0f)),
			VertexPositionTexture(XMFLOAT3(1.0f, -1.0f,  1.0f) , XMFLOAT2(1.0f, 1.0f)),

			// negative z
			VertexPositionTexture(XMFLOAT3(1.0f, -1.0f, -1.0f) , XMFLOAT2(0.0f, 1.0f)),
			VertexPositionTexture(XMFLOAT3(1.0f,  1.0f, -1.0f) , XMFLOAT2(0.0f, 0.0f)),
			VertexPositionTexture(XMFLOAT3(-1.0f,  1.0f, -1.0f) , XMFLOAT2(1.0f, 0.0f)),
			VertexPositionTexture(XMFLOAT3(-1.0f, -1.0f, -1.0f) , XMFLOAT2(1.0f, 1.0f)),
		};

		const SkyBox::IndexType SkyBox::CubeIndices[SkyBox::IndicesCount] =
		{
			0, 2, 1,
			0, 3, 2,
			// negative x
			4, 6, 5,
			4, 7, 6,
			// positive y
			8,10, 9,
			8,11,10,
			// negative y
			12,14,13,
			12,15,14,
			// positive z
			16,18,17,
			16,19,18,
			// negative z
			20,22,21,
			20,23,22,
		};

		//const FloorPlane::VertexType FloorPlane::Vertices[FloorPlane::VerticesCount] =
		//{
		//	VertexPositionNormalTexture(XMFLOAT3(1.0f, 0.0f, -1.0f) , XMFLOAT3(0.0f,1.0f,0.0f) , XMFLOAT2(0.0f, 1.0f)),
		//	VertexPositionNormalTexture(XMFLOAT3(-1.0f, 0.0f, -1.0f) , XMFLOAT3(0.0f,1.0f,0.0f) , XMFLOAT2(0.0f, 0.0f)),
		//	VertexPositionNormalTexture(XMFLOAT3(-1.0f, 0.0f,  1.0f) , XMFLOAT3(0.0f,1.0f,0.0f) , XMFLOAT2(1.0f, 0.0f)),
		//	VertexPositionNormalTexture(XMFLOAT3(1.0f, 0.0f,  1.0f) , XMFLOAT3(0.0f,1.0f,0.0f) , XMFLOAT2(1.0f, 1.0f)),
		//};

		//const FloorPlane::IndexType FloorPlane::Indices[FloorPlane::IndicesCount] =
		//{
		//	0,2,1,
		//	0,3,2,
		//};

		//FloorPlane::FloorPlane(ID3D11Device* pDevice, const ICamera* pCamera, const std::wstring &TextureFile)
		//	: BasicMesh(pDevice, Vertices, VerticesCount, Indices, IndicesCount)
		//	, m_pCamera(pCamera)
		//	, m_pEffect(new BasicEffect(pDevice))
		//	, m_pSamplerState(DirectX::CreateSamplerState(pDevice))

		//{
		//	HRESULT hr = CreateDDSTextureFromFile(pDevice, TextureFile.c_str(), m_pTexture.GetAddressOf(), m_pTextureView.GetAddressOf());
		//	ThrowIfFailed(hr);

		//	m_pEffect->SetLightingEnabled(false);
		//	m_pEffect->EnableDefaultLighting();
		//	m_pEffect->SetVertexColorEnabled(false);
		//	m_pEffect->SetFogEnabled(true);
		//	m_pEffect->SetTextureEnabled(true);
		//	m_pEffect->SetTexture(m_pTextureView.Get());
		//	m_pEffect->SetPerPixelLighting(true);
		//	m_pEffect->SetWorld(XMMatrixScaling(5.0f, 5.0f, 5.0f));

		//	void const* shaderByteCode;
		//	size_t byteCodeLength;
		//	m_pEffect->GetVertexShaderBytecode(&shaderByteCode, &byteCodeLength);

		//	ThrowIfFailed(
		//		pDevice->CreateInputLayout(VertexType::InputElements, VertexType::InputElementCount, shaderByteCode, byteCodeLength,
		//		m_pInputLayout.GetAddressOf()));
		//}

		//void FloorPlane::SetFloorTexture(ID3D11ShaderResourceView* pTextureSRV)
		//{
		//	m_pEffect->SetTexture(pTextureSRV);
		//}


		//void FloorPlane::Render(ID3D11DeviceContext* pDeviceContext)
		//{
		//	m_pEffect->SetView(m_pCamera->GetViewMatrix());
		//	m_pEffect->SetProjection(m_pCamera->GetProjectionMatrix());
		//	m_pEffect->Apply(pDeviceContext);
		//	pDeviceContext->IASetInputLayout(m_pInputLayout.Get());
		//	auto pSampler = m_pSamplerState.Get();
		//	pDeviceContext->PSSetSamplers(0, 1, &pSampler);
		//	BasicMesh::Render(pDeviceContext);
		//}

		//void FloorPlane::SetFloorPlaneEquation(DirectX::FXMVECTOR PlaneEquation)
		//{
		//	XMVECTOR qRot = DirectX::XMQuaternionRotationVectorToVector(g_XMIdentityR1, PlaneEquation);
		//	XMFLOAT4A Equation;
		//	XMStoreFloat4A(&Equation, PlaneEquation);
		//	XMVECTOR vDis = -(Equation.w / Equation.y) * g_XMIdentityR1;
		//	XMMATRIX World = XMMatrixScaling(5.0f, 5.0f, 5.0f) * XMMatrixRotationQuaternion(qRot) * XMMatrixTranslationFromVector(vDis);
		//	m_pEffect->SetWorld(World);
		//}

		//FloorPlane::~FloorPlane()
		//{
		//}


		SkyBox::~SkyBox(void)
		{
		}

		void XM_CALLCONV SkyBox::UpdateViewMatrix(DirectX::FXMMATRIX view)
		{
			XMMATRIX View = XMMatrixTranspose(view);
			// Last column of View Inverse is camera's position
			View.r[3] = g_XMIdentityR3;
			m_pEffect->SetView(XMMatrixTranspose(View));
		}

		void XM_CALLCONV SkyBox::UpdateProjectionMatrix(DirectX::FXMMATRIX projection)
		{
			m_pEffect->SetProjection(projection);
		}

		SkyBox::SkyBox(ID3D11Device* pDevice, const std::wstring(&TextureFiles)[6])
			: m_pCubeTexture(new CubeTexture(pDevice, TextureFiles))
			, m_pEffect(std::make_shared<BasicEffect>(pDevice))
		{
			m_pEffect->SetLightingEnabled(false);
			m_pEffect->SetTextureEnabled(true);
			m_pEffect->SetVertexColorEnabled(false);
			m_pEffect->SetFogEnabled(false);

			Mesh::CreateDeviceResources<VertexType,IndexType>(pDevice, CubeVertices, VerticesCount, CubeIndices, IndicesCount);
		}

		//template <class VertexType , class IndexType>
		//void TessallateTrianlges(std::vector<VertexType>& InputVerteices , std::vector<IndexType>& InputIndices , std::vector<VertexType>& OutputVerteices , std::vector<IndexType>& OutputIndices , std::function<VertexType,VertexType,VertexType>)
		//{

		//}


		void SkyBox::Render(ID3D11DeviceContext* pDeviceContext)
		{
			m_pEffect->SetWorld(XMMatrixScaling(20.0f, 20.0f, 20.0f));

			pDeviceContext->IASetInputLayout(pInputLayout.Get());

			unsigned int stride = sizeof(VertexType);
			unsigned int offset = 0;
			auto pBuffer = pVertexBuffer.Get();
			pDeviceContext->IASetVertexBuffers(0, 1, &pBuffer, &stride, &offset);
			pDeviceContext->IASetIndexBuffer(pIndexBuffer.Get(), IndexFormat, 0);
			pDeviceContext->IASetPrimitiveTopology(PrimitiveType);

			for (int i = 0; i < 6; i++)
			{
				m_pEffect->SetTexture(m_pCubeTexture->at(i));
				m_pEffect->Apply(pDeviceContext);
				pDeviceContext->DrawIndexed(6, 6 * i, 0);
			}

		}
	}


}