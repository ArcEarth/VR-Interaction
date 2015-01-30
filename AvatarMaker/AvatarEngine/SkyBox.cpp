#include "stdafx.h"
#include "SkyBox.h"
#include <DDSTextureLoader.h>

namespace DirectX
{
	class CubeTexture
	{
	public:
		enum Faces
		{
			Positive_X = 0,
			Negitive_X = 1,
			Positive_Y = 2,
			Negitive_Y = 3,
			Positive_Z = 4,
			Negitive_Z = 5,
			Front	= 5,
			Back	= 4,
			Left	= 0,
			Right	= 1,
			Top		= 2,
			Bottom	= 3,
		};

		void Initialize(ID3D11Device* pDevice ,const std::wstring (&TextureFiles)[6])
		{
			HRESULT hr;
			for (int i = 0; i < 6; i++)
			{
				hr = DirectX::CreateDDSTextureFromFile(pDevice,TextureFiles[i].c_str(),&(m_pTextures[i]),&(m_pTextureView[i]));
				if (FAILED(hr)) throw;
			}
		}

		CubeTexture(ID3D11Device* pDevice ,const std::wstring (&TextureFiles)[6])
		{
			Initialize(pDevice,TextureFiles);
		}

		CubeTexture()
		{}

		~CubeTexture()
		{}

		ID3D11ShaderResourceView* operator[] (unsigned int face)
		{
			return m_pTextureView[face];
		}

		ID3D11ShaderResourceView* at(unsigned int face)
		{
			return m_pTextureView[face];
		}
		ID3D11ShaderResourceView* const* ResourcesView()
		{
			return m_pTextureView;
		}


		ID3D11Resource* m_pTextures[6];
		ID3D11ShaderResourceView* m_pTextureView[6];
	};

	const SkyBox::VertexType SkyBox::CubeVertices[24] = 
	{
		// positive x
		VertexPositionTexture(XMFLOAT3( 1.0f, -1.0f,  1.0f) , XMFLOAT2(0.0f, 1.0f)),
		VertexPositionTexture(XMFLOAT3( 1.0f,  1.0f,  1.0f) , XMFLOAT2(0.0f, 0.0f)),
		VertexPositionTexture(XMFLOAT3( 1.0f,  1.0f, -1.0f) , XMFLOAT2(1.0f, 0.0f)),
		VertexPositionTexture(XMFLOAT3( 1.0f, -1.0f, -1.0f) , XMFLOAT2(1.0f, 1.0f)),

		// negative x
		VertexPositionTexture(XMFLOAT3(-1.0f, -1.0f, -1.0f) , XMFLOAT2(0.0f, 1.0f)),
		VertexPositionTexture(XMFLOAT3(-1.0f,  1.0f, -1.0f) , XMFLOAT2(0.0f, 0.0f)),
		VertexPositionTexture(XMFLOAT3(-1.0f,  1.0f,  1.0f) , XMFLOAT2(1.0f, 0.0f)),
		VertexPositionTexture(XMFLOAT3(-1.0f, -1.0f,  1.0f) , XMFLOAT2(1.0f, 1.0f)),

		// positive y
		VertexPositionTexture(XMFLOAT3(-1.0f,  1.0f,  1.0f) , XMFLOAT2(0.0f, 1.0f)),
		VertexPositionTexture(XMFLOAT3(-1.0f,  1.0f, -1.0f) , XMFLOAT2(0.0f, 0.0f)), 
		VertexPositionTexture(XMFLOAT3( 1.0f,  1.0f, -1.0f) , XMFLOAT2(1.0f, 0.0f)),
		VertexPositionTexture(XMFLOAT3( 1.0f,  1.0f,  1.0f) , XMFLOAT2(1.0f, 1.0f)),

		// negative y
		VertexPositionTexture(XMFLOAT3( 1.0f, -1.0f, -1.0f) , XMFLOAT2(0.0f, 1.0f)),
		VertexPositionTexture(XMFLOAT3(-1.0f, -1.0f, -1.0f) , XMFLOAT2(0.0f, 0.0f)),
		VertexPositionTexture(XMFLOAT3(-1.0f, -1.0f,  1.0f) , XMFLOAT2(1.0f, 0.0f)),
		VertexPositionTexture(XMFLOAT3( 1.0f, -1.0f,  1.0f) , XMFLOAT2(1.0f, 1.0f)),

		// positive z
		VertexPositionTexture(XMFLOAT3(-1.0f, -1.0f,  1.0f) , XMFLOAT2(0.0f, 1.0f)),
		VertexPositionTexture(XMFLOAT3(-1.0f,  1.0f,  1.0f) , XMFLOAT2(0.0f, 0.0f)),
		VertexPositionTexture(XMFLOAT3( 1.0f,  1.0f,  1.0f) , XMFLOAT2(1.0f, 0.0f)),
		VertexPositionTexture(XMFLOAT3( 1.0f, -1.0f,  1.0f) , XMFLOAT2(1.0f, 1.0f)),

		// negative z
		VertexPositionTexture(XMFLOAT3( 1.0f, -1.0f, -1.0f) , XMFLOAT2(0.0f, 1.0f)),
		VertexPositionTexture(XMFLOAT3( 1.0f,  1.0f, -1.0f) , XMFLOAT2(0.0f, 0.0f)),
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

	const FloorPlane::VertexType FloorPlane::Vertices[FloorPlane::VerticesCount] = 
	{
		VertexPositionNormalTexture(XMFLOAT3( 1.0f, 0.0f, -1.0f) , XMFLOAT3(0.0f,1.0f,0.0f) , XMFLOAT2(0.0f, 1.0f)),
		VertexPositionNormalTexture(XMFLOAT3(-1.0f, 0.0f, -1.0f) , XMFLOAT3(0.0f,1.0f,0.0f) , XMFLOAT2(0.0f, 0.0f)),
		VertexPositionNormalTexture(XMFLOAT3(-1.0f, 0.0f,  1.0f) , XMFLOAT3(0.0f,1.0f,0.0f) , XMFLOAT2(1.0f, 0.0f)),
		VertexPositionNormalTexture(XMFLOAT3( 1.0f, 0.0f,  1.0f) , XMFLOAT3(0.0f,1.0f,0.0f) , XMFLOAT2(1.0f, 1.0f)),
	};

	const FloorPlane::IndexType FloorPlane::Indices[FloorPlane::IndicesCount] = 
	{
		0,2,1,
		0,3,2,
	};

	FloorPlane::FloorPlane(ID3D11Device* pDevice , const ICamera* pCamera , const std::wstring &TextureFile)
		: BasicMesh(pDevice,Vertices,VerticesCount,Indices,IndicesCount)
		, m_pCamera(pCamera)
		, m_pEffect(new BasicEffect(pDevice))
		, m_pSamplerState(DirectX::CreateSamplerState(pDevice))

	{
		HRESULT hr = CreateDDSTextureFromFile(pDevice,TextureFile.c_str(),m_pTexture.GetAddressOf(),m_pTextureView.GetAddressOf());
		ThrowIfFailed(hr);

		m_pEffect->SetLightingEnabled(false);
		m_pEffect->EnableDefaultLighting();
		m_pEffect->SetVertexColorEnabled(false);
		m_pEffect->SetFogEnabled(true);
		m_pEffect->SetTextureEnabled(true);
		m_pEffect->SetTexture(m_pTextureView.Get());
		m_pEffect->SetPerPixelLighting(true);
		m_pEffect->SetWorld(XMMatrixScaling(5.0f,5.0f,5.0f));

		void const* shaderByteCode;
		size_t byteCodeLength;
		m_pEffect->GetVertexShaderBytecode(&shaderByteCode, &byteCodeLength);

		ThrowIfFailed(
			pDevice->CreateInputLayout(VertexType::InputElements,VertexType::InputElementCount,shaderByteCode, byteCodeLength,
			m_pInputLayout.GetAddressOf()));
	}

	void FloorPlane::SetFloorTexture(ID3D11ShaderResourceView* pTextureSRV)
	{
		m_pEffect->SetTexture(pTextureSRV);
	}


	void FloorPlane::Render(ID3D11DeviceContext* pDeviceContext)
	{
		m_pEffect->SetView(m_pCamera->GetViewMatrix());
		m_pEffect->SetProjection(m_pCamera->GetProjectionMatrix());
		m_pEffect->Apply(pDeviceContext);
		pDeviceContext->IASetInputLayout(m_pInputLayout.Get());
		auto pSampler = m_pSamplerState.Get();
		pDeviceContext->PSSetSamplers(0,1,&pSampler);
		BasicMesh::Render(pDeviceContext);
	}

	void FloorPlane::SetFloorPlaneEquation(DirectX::FXMVECTOR PlaneEquation)
	{
		XMVECTOR qRot = DirectX::XMQuaternionRotationVectorToVector(g_XMIdentityR1,PlaneEquation);
		XMFLOAT4A Equation;
		XMStoreFloat4A(&Equation,PlaneEquation);
		XMVECTOR vDis = -(Equation.w / Equation.y) * g_XMIdentityR1;
		XMMATRIX World =  XMMatrixScaling(5.0f,5.0f,5.0f) * XMMatrixRotationQuaternion(qRot) * XMMatrixTranslationFromVector(vDis);
		m_pEffect->SetWorld(World);
	}

	FloorPlane::~FloorPlane()
	{
	}


	SkyBox::~SkyBox(void)
	{
	}

	SkyBox::SkyBox(ID3D11Device* pDevice , const ICamera* pCamera , const std::wstring (&TextureFiles)[6])
		: BasicMesh(pDevice,CubeVertices,VerticesCount,CubeIndices,IndicesCount)
		, m_pCubeTexture(new CubeTexture(pDevice,TextureFiles))
		, m_pCamera(pCamera)
		, m_pEffect(new BasicEffect(pDevice))
	{
		m_pEffect->SetLightingEnabled(false);
		m_pEffect->SetVertexColorEnabled(false);
		m_pEffect->SetFogEnabled(false);
		m_pEffect->SetTextureEnabled(true);
		void const* shaderByteCode;
		size_t byteCodeLength;

		m_pEffect->GetVertexShaderBytecode(&shaderByteCode, &byteCodeLength);

		ThrowIfFailed(
			pDevice->CreateInputLayout(VertexType::InputElements,
			VertexType::InputElementCount,
			shaderByteCode, byteCodeLength,
			m_pInputLayout.GetAddressOf()));
	}

	//template <class VertexType , class IndexType>
	//void TessallateTrianlges(std::vector<VertexType>& InputVerteices , std::vector<IndexType>& InputIndices , std::vector<VertexType>& OutputVerteices , std::vector<IndexType>& OutputIndices , std::function<VertexType,VertexType,VertexType>)
	//{

	//}


	void SkyBox::Render(ID3D11DeviceContext* pDeviceContext)
	{
		pDeviceContext->IASetInputLayout(m_pInputLayout.Get());
		m_pEffect->SetWorld(XMMatrixScaling(20.0f,20.0f,20.0f) * XMMatrixTranslationFromVector(m_pCamera->GetPosition()));
		m_pEffect->SetView(m_pCamera->GetViewMatrix());
		m_pEffect->SetProjection(m_pCamera->GetProjectionMatrix());

		unsigned int stride = sizeof(VertexType);
		unsigned int offset = 0;
		auto pBuffer = m_pVertexBuffer.Get();
		pDeviceContext->IASetVertexBuffers(0,1,&pBuffer,&stride,&offset);
		pDeviceContext->IASetIndexBuffer(m_pIndexBuffer.Get(),ExtractDXGIFormat<IndexType>::value,0);
		pDeviceContext->IASetPrimitiveTopology(PrimitiveType);

		for (int i = 0; i < 6; i++)
		{
			m_pEffect->SetTexture(m_pCubeTexture->at(i));
			m_pEffect->Apply(pDeviceContext);
			pDeviceContext->DrawIndexed(6,6*i,0);
		}

	}


}