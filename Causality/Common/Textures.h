#ifndef DYNAMIC_TEXTURE_H
#define DYNAMIC_TEXTURE_H

#include <d3d11_1.h>
#include <DirectXMath.h>
#include <memory>
#include <wrl\client.h>
#include <string>

namespace DirectX{


	class ITexture
	{
	public:
		ITexture(ITexture&& Src);
		ITexture& operator=(ITexture&& Src);

		virtual ~ITexture();

		static std::unique_ptr<ITexture> CreateFromDDSFile( _In_ ID3D11Device* pDevice , _In_z_ const wchar_t* szFileName ,
			_In_opt_ size_t maxsize = 0, 
			_In_opt_ D3D11_USAGE usage = D3D11_USAGE_DEFAULT,
			_In_opt_ unsigned int bindFlags = D3D11_BIND_SHADER_RESOURCE,
			_In_opt_ unsigned int cpuAccessFlags = 0,
			_In_opt_ unsigned int miscFlags = 0,
			_In_opt_ bool forceSRGB = false
			);

		void SaveAsDDSFile(_In_ ID3D11DeviceContext *pDeviceContext , _In_z_ const wchar_t* szFileName);

		//static std::unique_ptr<ITexture> CreateFromDDSMemory(_In_ ID3D11Device* pDevice,
		//	_In_reads_bytes_(ddsDataSize) const uint8_t* ddsData,
		//	_In_ size_t ddsDataSize,
		//	_In_opt_ size_t maxsize = 0, 
		//	_In_opt_ D3D11_USAGE usage = D3D11_USAGE_DEFAULT,
		//	_In_opt_ unsigned int bindFlags = D3D11_BIND_SHADER_RESOURCE,
		//	_In_opt_ unsigned int cpuAccessFlags = 0,
		//	_In_opt_ unsigned int miscFlags = 0,
		//	_In_opt_ bool forceSRGB = false
		//	);


		DXGI_FORMAT Format() const
		{
			D3D11_SHADER_RESOURCE_VIEW_DESC Desc;
			m_pShaderResourceView->GetDesc(&Desc);
			return Desc.Format;
		}

		size_t MipMapLevel() const
		{
			D3D11_SHADER_RESOURCE_VIEW_DESC Desc;

			m_pShaderResourceView->GetDesc(&Desc);

			auto type = Desc.ViewDimension;

			switch (type)
			{
			case D3D11_SRV_DIMENSION_TEXTURE1D:
				return Desc.Texture1D.MipLevels;
				break;
			case D3D11_SRV_DIMENSION_TEXTURE2D:
				return Desc.Texture2D.MipLevels;
				break;
			case D3D11_SRV_DIMENSION_TEXTURE3D:
				return Desc.Texture3D.MipLevels;
				break;
			case D3D11_SRV_DIMENSION_TEXTURECUBE:
				return Desc.TextureCube.MipLevels;
				break;
			default:
				break;
			}
			return 0;
		}

		D3D11_RESOURCE_DIMENSION Dimension() const
		{
			D3D11_RESOURCE_DIMENSION Demension;
			m_pResource->GetType(&Demension);
			return Demension;
		}

		virtual ID3D11Resource*	Resource() const
		{
			return m_pResource.Get();
		}


		operator ID3D11ShaderResourceView* ()
		{
			return m_pShaderResourceView.Get();
		}

		ID3D11ShaderResourceView* ShaderResourceView() const
		{ 
			return m_pShaderResourceView.Get();
		}

	protected:

		Microsoft::WRL::ComPtr<ID3D11Resource>				m_pResource;
		Microsoft::WRL::ComPtr<ID3D11ShaderResourceView>	m_pShaderResourceView;

	protected:
		ITexture();
	private:
		ITexture(const ITexture&);
		void operator= (const ITexture&);
	};

	class Texture2D
		: public ITexture
	{
	public:
		Texture2D(Texture2D &&);
		Texture2D& operator=(Texture2D &&);
		static std::unique_ptr<Texture2D> CreateFromWICFile( _In_ ID3D11Device* pDevice ,
			_In_ ID3D11DeviceContext* pDeviceContext ,
			_In_z_ const wchar_t* szFileName ,
			_In_opt_ size_t maxsize = 0, 
			_In_opt_ D3D11_USAGE usage = D3D11_USAGE_DEFAULT,
			_In_opt_ unsigned int bindFlags = D3D11_BIND_SHADER_RESOURCE,
			_In_opt_ unsigned int cpuAccessFlags = 0,
			_In_opt_ unsigned int miscFlags = 0,
			_In_opt_ bool forceSRGB = false
			);

		static std::unique_ptr<Texture2D> CreateFromWICMemory(_In_ ID3D11Device* pDevice,
			_In_ ID3D11DeviceContext* pDeviceContext ,
			_In_reads_bytes_(wicDataSize) const uint8_t* wicData,
			_In_ size_t wicDataSize,
			_In_opt_ size_t maxsize = 0, 
			_In_opt_ D3D11_USAGE usage = D3D11_USAGE_DEFAULT,
			_In_opt_ unsigned int bindFlags = D3D11_BIND_SHADER_RESOURCE,
			_In_opt_ unsigned int cpuAccessFlags = 0,
			_In_opt_ unsigned int miscFlags = 0,
			_In_opt_ bool forceSRGB = false
			);

		//operator ID3D11Texture2D* ();

		~Texture2D();

	public:
		const D3D11_TEXTURE2D_DESC& Description() const
		{
			return m_TextureDescription;
		}

		size_t Width() const
		{ 
			return m_TextureDescription.Width;
		}
		size_t Height() const
		{
			return m_TextureDescription.Height;
		}
		XMUINT2 Bounds() const
		{
			return XMUINT2(m_TextureDescription.Width,m_TextureDescription.Height);
		}

		DXGI_FORMAT Format() const
		{
			return m_TextureDescription.Format;
		}

		size_t MipMapLevel() const
		{
			return m_TextureDescription.MipLevels;
		}

		// Warning! This one is not safe?
		ID3D11Texture2D *Texture() const
		{
			Microsoft::WRL::ComPtr<ID3D11Texture2D> pTex;
			m_pResource.As(&pTex);
			return pTex.Get();
		}

		operator ID3D11ShaderResourceView* ()
		{
			return m_pShaderResourceView.Get();
		}

		size_t SizeInByte() const;

		void CopyFrom(ID3D11DeviceContext *pContext,const Texture2D* pSource);

	public:
		Texture2D( _In_ ID3D11Device* pDevice, _In_ unsigned int Width, _In_ unsigned int Height,
			_In_opt_ unsigned int MipMapLevel = 1,
			_In_opt_ DXGI_FORMAT SurfaceFormat = DXGI_FORMAT_R32G32B32A32_FLOAT ,
			_In_opt_ D3D11_USAGE usage = D3D11_USAGE_DEFAULT,
			_In_opt_ unsigned int bindFlags = D3D11_BIND_SHADER_RESOURCE,
			_In_opt_ unsigned int cpuAccessFlags = 0,
			_In_opt_ unsigned int miscFlags = 0,
			_In_opt_ unsigned int multisamplesCount = 1,
			_In_opt_ unsigned int multisamplesQuality = 0
			);
		Texture2D(ID3D11Device* pDevice, const D3D11_TEXTURE2D_DESC *pDesc , const D3D11_SUBRESOURCE_DATA *pInitialData = nullptr);
		Texture2D(ID3D11Texture2D* pTexture , ID3D11ShaderResourceView* pResourceView = nullptr);
		Texture2D(ID3D11Resource* pResource , ID3D11ShaderResourceView* pResourceView = nullptr);
		Texture2D(ID3D11Device* pDevice , ID3D11Resource* pResouce);
		Texture2D();

	protected:
		Microsoft::WRL::ComPtr<ID3D11Texture2D>	m_pTexture;
		D3D11_TEXTURE2D_DESC					m_TextureDescription;

	private:
		Texture2D(const Texture2D&);
		void operator= (const Texture2D&);

		friend ITexture;
	};

	class DynamicTexture2D
		: public Texture2D
	{
	public:
		DynamicTexture2D( _In_ ID3D11Device* pDevice, _In_ unsigned int Width, _In_ unsigned int Height,
			_In_opt_ DXGI_FORMAT Format = DXGI_FORMAT_R8G8B8A8_UNORM);
		~DynamicTexture2D();


		template<typename _T>
		void SetData(ID3D11DeviceContext* pContext , const _T* Raw_Data)
		{
			SetData(pContext,Raw_Data,sizeof(_T));
		}

		template<>
		void SetData<void>(ID3D11DeviceContext* pContext , const void* Raw_Data)
		{
			SetData(pContext,Raw_Data,0);
		}



	protected:
		void SetData(ID3D11DeviceContext* pContext , const void* Raw_Data , size_t element_size);
	};

	class RenderTargetTexture2D
		: public Texture2D
	{
	public:
		RenderTargetTexture2D();
		RenderTargetTexture2D(RenderTargetTexture2D &&);
		RenderTargetTexture2D& operator=(RenderTargetTexture2D &&);
		RenderTargetTexture2D(_In_ ID3D11Device* pDevice, _In_ unsigned int Width, _In_ unsigned int Height,
			_In_opt_ DXGI_FORMAT Format = DXGI_FORMAT_R8G8B8A8_UNORM , _In_opt_ bool Shared = false);
		RenderTargetTexture2D( ID3D11Texture2D* pTexture , ID3D11RenderTargetView* pRenderTargetView , ID3D11ShaderResourceView* pShaderResouceView , const D3D11_VIEWPORT *pViewport = nullptr);
		~RenderTargetTexture2D();


		operator ID3D11RenderTargetView* ()
		{
			return m_pRenderTargetView.Get();
		}

		operator ID3D11ShaderResourceView* ()
		{
			return m_pShaderResourceView.Get();
		}

		ID3D11RenderTargetView* RenderTargetView()
		{
			return m_pRenderTargetView.Get();
		}

		/// <summary>
		/// return a Const reference to the ViewPort binding this render target texture for RS stage.
		/// </summary>
		/// <returns> the return value is default to be (0,0) at left-top corner , while min-max depth is (0,1000) </returns>
		inline const D3D11_VIEWPORT& ViewPort() const
		{
			return m_Viewport;
		}

		/// <summary>
		/// return a Reference to the ViewPort binding this render target texture for RS stage.
		/// </summary>
		/// <returns> the return value is default to be (0,0) at left-top corner , while min-max depth is (0,1000) </returns>
		inline D3D11_VIEWPORT& ViewPort()
		{
			return m_Viewport;
		}

		/// <summary>
		/// return the Same Render Target as this , but with different Viewport.
		/// </summary>
		/// <returns> the return value is default to be (0,0) at left-top corner , while min-max depth is (0,1000) </returns>
		RenderTargetTexture2D Subtexture(const D3D11_VIEWPORT *pViewport)
		{
			Microsoft::WRL::ComPtr<ID3D11Texture2D> pTexture;
			m_pResource.As<ID3D11Texture2D>(&pTexture);
			return RenderTargetTexture2D(pTexture.Get(), m_pRenderTargetView.Get(), m_pShaderResourceView.Get(), pViewport);
		}

		/// <summary>
		/// Sets as render target with default no DepthStencil.
		/// </summary>
		/// <param name="pDeviceContext">The pointer to device context.</param>
		/// <param name="pDepthStencil">The pointer to depth stencil view.</param>
		void SetAsRenderTarget(ID3D11DeviceContext* pDeviceContext, ID3D11DepthStencilView* pDepthStencil = nullptr)
		{
			auto pTargetView = RenderTargetView();
			pDeviceContext->RSSetViewports(1,&m_Viewport);
			pDeviceContext->OMSetRenderTargets(1,&pTargetView,pDepthStencil);
		}

		void Clear(ID3D11DeviceContext *pDeviceContext , FXMVECTOR Color = g_XMIdentityR3)
		{
			pDeviceContext->ClearRenderTargetView(m_pRenderTargetView.Get(),reinterpret_cast<const float*>(&Color));
		}

	protected:
		Microsoft::WRL::ComPtr<ID3D11RenderTargetView>			m_pRenderTargetView;
		D3D11_VIEWPORT											m_Viewport;
	};

	/// <summary>
	/// A Texture class which can not be directly Access by GPU , but use for transform Data from GPU and CPU 
	///	Especially useful for retrieving texture data from GPU
	/// </summary>
	class StagingTexture2D
		: public Texture2D
	{
	public:
		StagingTexture2D(ID3D11Device* pDevice, unsigned int Width, unsigned int Height , _In_ DXGI_FORMAT Format ,  _In_opt_ unsigned int cpuAccessFlags = D3D11_CPU_ACCESS_WRITE | D3D11_CPU_ACCESS_READ);
		StagingTexture2D(ID3D11DeviceContext* pContext, const Texture2D* pSource ,  _In_opt_ unsigned int cpuAccessFlags = D3D11_CPU_ACCESS_READ);

		//static std::unique_ptr<StagingTexture2D> CreateFromTexture2D(ID3D11DeviceContext* pContext, const Texture2D* pSource ,  _In_opt_ unsigned int cpuAccessFlags = D3D11_CPU_ACCESS_READ);

		std::unique_ptr<uint8_t> GetData(ID3D11DeviceContext *pContext);
		//void GetData(ID3D11DeviceContext *pContext , void *pTarget);

	protected:
		operator ID3D11ShaderResourceView* () = delete;
		ID3D11ShaderResourceView* ShaderResourceView() = delete;

		StagingTexture2D(ID3D11Texture2D* pTexture);
	};


	class DepthStencilBuffer
		: public Texture2D
	{
	public:
		DepthStencilBuffer();
		DepthStencilBuffer(DepthStencilBuffer&&);
		DepthStencilBuffer& operator = (DepthStencilBuffer&&);
		DepthStencilBuffer(ID3D11Device* pDevice, unsigned int Width, unsigned int Height, _In_opt_ DXGI_FORMAT Format = DXGI_FORMAT_D24_UNORM_S8_UINT);
		~DepthStencilBuffer();

		operator ID3D11DepthStencilView*()
		{
			return m_pDepthStencilView.Get();
		}

		ID3D11DepthStencilView* DepthStencilView()
		{
			return m_pDepthStencilView.Get();
		}

		void Clear(ID3D11DeviceContext *pDeviceContext , uint32_t ClearFlag = D3D11_CLEAR_DEPTH || D3D11_CLEAR_STENCIL,float Depth = 1.0f , uint8_t Stencil = 0U)
		{
			pDeviceContext->ClearDepthStencilView(m_pDepthStencilView.Get(),ClearFlag,Depth,Stencil);
		}

	protected:
		operator ID3D11ShaderResourceView* ();
		ID3D11ShaderResourceView* ShaderResourceView();

		Microsoft::WRL::ComPtr<ID3D11DepthStencilView>			m_pDepthStencilView;
	};

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
			Front = 5,
			Back = 4,
			Left = 0,
			Right = 1,
			Top = 2,
			Bottom = 3,
		};

		void Initialize(ID3D11Device* pDevice, const std::wstring(&TextureFiles)[6]);

		CubeTexture(ID3D11Device* pDevice, const std::wstring(&TextureFiles)[6])
		{
			Initialize(pDevice, TextureFiles);
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

}

#endif