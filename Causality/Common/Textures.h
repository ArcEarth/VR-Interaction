#ifndef DYNAMIC_TEXTURE_H
#define DYNAMIC_TEXTURE_H

#include <d3d11_1.h>
#include <DirectXMath.h>
#include <memory>
#include <wrl\client.h>
#include <string>

namespace DirectX{

	//class DeviceResource
	//{

	//protected:
	//	std::string											m_Source;
	//};


	class Texture
	{
	public:
		enum TextureType
		{
			Texture_1D,
			Texture_1D_Array,
			Texture_2D,
			Texture_2D_Array,
			Texture_3D,
			TextureCube,
		};

	public:
		Texture();
		Texture(nullptr_t)
		{}

		Texture(Texture&& Src);
		Texture(const Texture& rhs)
		{
			*this = rhs;
		}
		Texture& operator= (Texture&&);
		Texture& operator= (const Texture& rhs)
		{
			m_pResource = rhs.m_pResource;
			m_pShaderResourceView = rhs.m_pShaderResourceView;
			return *this;
		}
		Texture(ID3D11Resource* pResource, ID3D11ShaderResourceView* pResourceView);

		virtual ~Texture();

		TextureType Type() const;

		static Texture CreateFromDDSFile( _In_ ID3D11Device* pDevice , _In_z_ const wchar_t* szFileName ,
			_In_opt_ size_t maxsize = 0, 
			_In_opt_ D3D11_USAGE usage = D3D11_USAGE_DEFAULT,
			_In_opt_ unsigned int bindFlags = D3D11_BIND_SHADER_RESOURCE,
			_In_opt_ unsigned int cpuAccessFlags = 0,
			_In_opt_ unsigned int miscFlags = 0,
			_In_opt_ bool forceSRGB = false
			);

		void SaveAsDDSFile(_In_ ID3D11DeviceContext *pDeviceContext , _In_z_ const wchar_t* szFileName);

		//static std::unique_ptr<Texture> CreateFromDDSMemory(_In_ ID3D11Device* pDevice,
		//	_In_reads_bytes_(ddsDataSize) const uint8_t* ddsData,
		//	_In_ size_t ddsDataSize,
		//	_In_opt_ size_t maxsize = 0, 
		//	_In_opt_ D3D11_USAGE usage = D3D11_USAGE_DEFAULT,
		//	_In_opt_ unsigned int bindFlags = D3D11_BIND_SHADER_RESOURCE,
		//	_In_opt_ unsigned int cpuAccessFlags = 0,
		//	_In_opt_ unsigned int miscFlags = 0,
		//	_In_opt_ bool forceSRGB = false
		//	);

		operator bool() const
		{
			return m_pResource != nullptr;
		}

		bool operator != (nullptr_t) const
		{
			return m_pResource != nullptr;
		}

		bool operator == (nullptr_t) const
		{
			return m_pResource == nullptr;
		}

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

		//const ID3D11Resource* Resource() const
		//{
		//	return const_cast<Texture*>(this)->Resource();
		//}

		operator ID3D11ShaderResourceView* () const
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
	};

	class Texture2D
		: public Texture
	{
	public:
		Texture2D(Texture2D &&);
		Texture2D(const Texture2D& rhs)
		{
			*this = rhs;
		}

		Texture2D& operator=(const Texture2D & rhs)
		{
			Texture::operator=(rhs);
			m_pTexture = rhs.m_pTexture;
			m_Description = rhs.m_Description;
			return *this;
		}

		Texture2D& operator=(Texture2D &&);

		static Texture2D CreateFromWICFile( _In_ ID3D11Device* pDevice ,
			_In_ ID3D11DeviceContext* pDeviceContext ,
			_In_z_ const wchar_t* szFileName ,
			_In_opt_ size_t maxsize = 0, 
			_In_opt_ D3D11_USAGE usage = D3D11_USAGE_DEFAULT,
			_In_opt_ unsigned int bindFlags = D3D11_BIND_SHADER_RESOURCE,
			_In_opt_ unsigned int cpuAccessFlags = 0,
			_In_opt_ unsigned int miscFlags = 0,
			_In_opt_ bool forceSRGB = false
			);

		static Texture2D CreateFromWICMemory(_In_ ID3D11Device* pDevice,
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
			return m_Description;
		}

		size_t Width() const
		{ 
			return m_Description.Width;
		}
		size_t Height() const
		{
			return m_Description.Height;
		}
		XMUINT2 Bounds() const
		{
			return XMUINT2(m_Description.Width,m_Description.Height);
		}

		bool IsMultiSampleEnabled() const;
		size_t MultiSampleCount() const;
		size_t MultiSampleQuality() const;

		DXGI_FORMAT Format() const
		{
			return m_Description.Format;
		}

		size_t MipMapLevel() const
		{
			return m_Description.MipLevels;
		}

		// Warning! This one is not safe?
		ID3D11Texture2D *Texture() const
		{
			return m_pTexture.Get();
		}

		D3D11_USAGE Usage() const
		{
			return m_Description.Usage;
		}

		bool IsCpuWritable() const;
		bool IsCpuReadable() const;
		bool IsCubeMap() const;

		operator ID3D11ShaderResourceView* () const
		{
			return m_pShaderResourceView.Get();
		}

		operator ID3D11Texture2D* () const
		{
			return m_pTexture.Get();
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
		Texture2D(ID3D11ShaderResourceView* pResourceView);
		Texture2D();

	protected:
		Microsoft::WRL::ComPtr<ID3D11Texture2D>	m_pTexture;
		D3D11_TEXTURE2D_DESC					m_Description;

		friend class Texture;
	};

	// Cpu Writable texture
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
		RenderTargetTexture2D(const RenderTargetTexture2D & rhs)
		{
			*this = rhs;
		}
		RenderTargetTexture2D& operator=(const RenderTargetTexture2D &rhs)
		{
			Texture2D::operator=(rhs);
			m_pRenderTargetView = rhs.m_pRenderTargetView;
			return *this;
		}

		RenderTargetTexture2D(_In_ ID3D11Device* pDevice, _In_ unsigned int Width, _In_ unsigned int Height,
			_In_opt_ DXGI_FORMAT Format = DXGI_FORMAT_R8G8B8A8_UNORM , _In_opt_ UINT MultiSampleCount = 1, _In_opt_ UINT MultiSampleQuality = 0, _In_opt_ bool Shared = false);
		RenderTargetTexture2D( ID3D11Texture2D* pTexture , ID3D11RenderTargetView* pRenderTargetView , ID3D11ShaderResourceView* pShaderResouceView);
		explicit RenderTargetTexture2D(ID3D11RenderTargetView* pRenderTargetView);

		~RenderTargetTexture2D();

		operator ID3D11RenderTargetView* ()
		{
			return m_pRenderTargetView.Get();
		}

		using Texture2D::operator ID3D11ShaderResourceView *;
		using Texture2D::operator ID3D11Texture2D *;

		ID3D11RenderTargetView* RenderTargetView()
		{
			return m_pRenderTargetView.Get();
		}

		void Clear(ID3D11DeviceContext *pDeviceContext , FXMVECTOR Color = g_XMIdentityR3)
		{
			DirectX::Color col = Color;
			if (m_pRenderTargetView)
				pDeviceContext->ClearRenderTargetView(m_pRenderTargetView.Get(),&col.x);
		}

	protected:
		Microsoft::WRL::ComPtr<ID3D11RenderTargetView>			m_pRenderTargetView;
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
		DepthStencilBuffer(const DepthStencilBuffer& rhs)
		{
			*this = rhs;
		}
		DepthStencilBuffer& operator = (const DepthStencilBuffer& rhs)
		{
			Texture2D::operator=(rhs);
			m_pDepthStencilView = rhs.m_pDepthStencilView;
			return *this;
		}

		DepthStencilBuffer(ID3D11Texture2D* pTexture, ID3D11DepthStencilView* pDSV);
		explicit DepthStencilBuffer(ID3D11DepthStencilView* pDSV);

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

		void Clear(ID3D11DeviceContext *pDeviceContext , uint32_t ClearFlag = D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL,float Depth = 1.0f , uint8_t Stencil = 0U)
		{
			pDeviceContext->ClearDepthStencilView(m_pDepthStencilView.Get(),ClearFlag,Depth,Stencil);
		}

		operator ID3D11ShaderResourceView* () = delete;
		ID3D11ShaderResourceView* ShaderResourceView() = delete;

	protected:
		Microsoft::WRL::ComPtr<ID3D11DepthStencilView>			m_pDepthStencilView;
	};

	class IRenderTarget
	{
		virtual void SetAsRenderTarget(ID3D11DeviceContext* pDeviceContext) = 0;
	};
	// Render Target is a collection of Depth Buffer, Color Buffer, Viewport
	class RenderTarget : public IRenderTarget
	{
	public:
		RenderTarget();

		RenderTarget(RenderTargetTexture2D& colorBuffer, DepthStencilBuffer& dsBuffer, const D3D11_VIEWPORT& viewPort);

		RenderTarget(ID3D11Device* pDevice, size_t width, size_t height);

		RenderTarget Subview(const D3D11_VIEWPORT *pViewport);
		/// <summary>
		/// return a Const reference to the ViewPort binding this render target texture for RS stage.
		/// </summary>
		/// <returns> the return value is default to be (0,0) at left-top corner , while min-max depth is (0,1000) </returns>
		inline const D3D11_VIEWPORT& ViewPort() const;

		void Clear(ID3D11DeviceContext* pContext, FXMVECTOR Color = g_XMIdentityR3);

		/// <summary>
		/// return a Reference to the ViewPort binding this render target texture for RS stage.
		/// </summary>
		/// <returns> the return value is default to be (0,0) at left-top corner , while min-max depth is (0,1000) </returns>
		D3D11_VIEWPORT& ViewPort();

		RenderTargetTexture2D& ColorBuffer();

		const RenderTargetTexture2D& ColorBuffer() const;

		DepthStencilBuffer& DepthBuffer();

		const DepthStencilBuffer& DepthBuffer() const;

		void SetDepthBuffer(DepthStencilBuffer& depthBuffer);

		/// <summary>
		/// Sets as render target with default no DepthStencil.
		/// </summary>
		/// <param name="pDeviceContext">The pointer to device context.</param>
		/// <param name="pDepthStencil">The pointer to depth stencil view.</param>
		void SetAsRenderTarget(ID3D11DeviceContext* pDeviceContext) override;

	private:
		RenderTargetTexture2D									m_ColorBuffer;
		DepthStencilBuffer										m_DepthStencilBuffer;
		D3D11_VIEWPORT											m_Viewport;
	};

	//class SimultaneousRenderTarget : public IRenderTarget
	//{
	//	void SetAsRenderTarget(ID3D11DeviceContext* pDeviceContext) override
	//	{
	//	}

	//	std::vector<D3D11_VIEWPORT>								m_Viewports;
	//	DepthStencilBuffer										m_DepthStencilBuffer;
	//};

	class CubeTexture : public Texture
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

		CubeTexture(ID3D11Resource* pResource, ID3D11ShaderResourceView* pResourceView);

		static CubeTexture CreateFromFile();

		void Initialize(ID3D11Device* pDevice, const std::wstring(&TextureFiles)[6]);

		CubeTexture(ID3D11Device* pDevice, const std::wstring(&TextureFiles)[6]);

		CubeTexture();

		~CubeTexture();

		ID3D11ShaderResourceView* operator[] (unsigned int face)
		{
			return m_pTextureView[face];
		}

		ID3D11ShaderResourceView* at(unsigned int face);
		ID3D11ShaderResourceView* const* ResourcesView();


		ID3D11Resource* m_pTextures[6];
		ID3D11ShaderResourceView* m_pTextureView[6];
	};

}

#endif