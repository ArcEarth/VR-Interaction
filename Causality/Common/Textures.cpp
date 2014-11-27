#include "Textures.h"
#include "DirectXHelper.h"
#include "DXGIFormatHelper.h"
#include <DDSTextureLoader.h>
#include <WICTextureLoader.h>
#include <algorithm>
#include <ScreenGrab.h>
#include <wincodec.h>

using namespace DirectX;
using namespace std;

ITexture::ITexture(ITexture&& Src)
	: m_pTexture(std::move(Src.m_pTexture))
	, m_pShaderResourceView(std::move(Src.ShaderResourceView()))
{
}

ITexture& ITexture::operator = (ITexture&& Src)
{
	m_pTexture = std::move(Src.m_pTexture);
	m_pShaderResourceView = std::move(Src.ShaderResourceView());
	return *this;
}

Texture2D::Texture2D(Texture2D &&Src)
	:ITexture(std::move(Src))
	, m_TextureDescription(Src.m_TextureDescription)
{
}

Texture2D& Texture2D::operator = (Texture2D &&Src)
{
	ITexture::operator=(std::move(Src));
	m_TextureDescription = Src.m_TextureDescription;
	return *this;
}


Texture2D::Texture2D(_In_ ID3D11Device* pDevice, _In_ unsigned int Width, _In_ unsigned int Height,
	_In_opt_ unsigned int MipMapLevel,
	_In_opt_ DXGI_FORMAT format,
	_In_opt_ D3D11_USAGE usage,
	_In_opt_ unsigned int bindFlags,
	_In_opt_ unsigned int cpuAccessFlags,
	_In_opt_ unsigned int miscFlags,
	_In_opt_ unsigned int multisamplesCount,
	_In_opt_ unsigned int multisamplesQuality
	)
{
	D3D11_TEXTURE2D_DESC &TextureDesc = m_TextureDescription;
	TextureDesc.Width = Width;
	TextureDesc.Height = Height;
	TextureDesc.MipLevels = MipMapLevel;
	TextureDesc.ArraySize = 1;
	TextureDesc.Format = format;
	TextureDesc.SampleDesc.Count = multisamplesCount;
	TextureDesc.SampleDesc.Quality = multisamplesQuality;
	TextureDesc.Usage = usage;
	TextureDesc.BindFlags = bindFlags;
	TextureDesc.CPUAccessFlags = cpuAccessFlags;
	TextureDesc.MiscFlags = miscFlags;

	ID3D11Texture2D *pTexture;
	HRESULT hr = pDevice->CreateTexture2D(&TextureDesc, NULL, &pTexture);
	DirectX::ThrowIfFailed(hr);
	m_pTexture = pTexture;

	if ((bindFlags & D3D11_BIND_DEPTH_STENCIL) && !(bindFlags & D3D11_BIND_SHADER_RESOURCE) && !(format == DXGI_FORMAT_R32_TYPELESS))
	{
		m_pShaderResourceView = nullptr;
		return;
	}

	D3D11_SHADER_RESOURCE_VIEW_DESC ShaderResourceViewDesc;
	ShaderResourceViewDesc.Format = TextureDesc.Format;
	ShaderResourceViewDesc.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2D;
	ShaderResourceViewDesc.Texture2D.MostDetailedMip = 0;
	ShaderResourceViewDesc.Texture2D.MipLevels = 1;

	hr = pDevice->CreateShaderResourceView(m_pTexture.Get(), &ShaderResourceViewDesc, &m_pShaderResourceView);
	DirectX::ThrowIfFailed(hr);
}

Texture2D::Texture2D(ID3D11Texture2D* pTexture, ID3D11ShaderResourceView* pResourceView)
{
	pTexture->GetDesc(&m_TextureDescription);
	m_pTexture = pTexture;
	m_pShaderResourceView = pResourceView;
}

Texture2D::Texture2D(ID3D11Resource* pResource, ID3D11ShaderResourceView* pResourceView)
{
	assert(pResource);
	ID3D11Texture2D *pTexture = nullptr;
	HRESULT hr = pResource->QueryInterface<ID3D11Texture2D>(&pTexture);
	DirectX::ThrowIfFailed(hr);
	pTexture->GetDesc(&m_TextureDescription);
	m_pShaderResourceView = pResourceView;
	m_pTexture = pTexture;

	pTexture->Release();
	//pResource->Release();
}

void Texture2D::CopyFrom(ID3D11DeviceContext *pContext, const Texture2D* pSource)
{
	assert(this != pSource);
	assert(this->Resource() != pSource->Resource());
	pContext->CopyResource(this->Resource(), pSource->Resource());
}



void DynamicTexture2D::SetData(ID3D11DeviceContext* pContext, const void* Raw_Data, size_t element_size)
{
	assert(element_size == 0 || element_size == DXGIFormatTraits::SizeofDXGIFormatInBytes(Format()));

	if (element_size == 0)
		element_size = DXGIFormatTraits::SizeofDXGIFormatInBytes(Format());


	D3D11_MAPPED_SUBRESOURCE Resouce;
	auto hr = pContext->Map(m_pTexture.Get(), 0, D3D11_MAP_WRITE_DISCARD, 0, &Resouce);
	DirectX::ThrowIfFailed(hr);

	memcpy(Resouce.pData, Raw_Data, element_size * Width() * Height());
	pContext->Unmap(m_pTexture.Get(), 0);
}

DynamicTexture2D::DynamicTexture2D(_In_ ID3D11Device* pDevice, _In_ unsigned int Width, _In_ unsigned int Height, _In_opt_ DXGI_FORMAT Format)
	: Texture2D(pDevice, Width, Height, 1, Format, D3D11_USAGE_DYNAMIC, D3D11_BIND_SHADER_RESOURCE, D3D11_CPU_ACCESS_WRITE)
{
}

RenderTargetTexture2D::RenderTargetTexture2D(_In_ ID3D11Device* pDevice, _In_ unsigned int Width, _In_ unsigned int Height,
	_In_opt_ DXGI_FORMAT Format, _In_opt_ bool Shared)
	: Texture2D(pDevice, Width, Height, 1, Format, D3D11_USAGE_DEFAULT, D3D11_BIND_RENDER_TARGET | D3D11_BIND_SHADER_RESOURCE, 0, Shared ? D3D11_RESOURCE_MISC_SHARED : 0,4,1)
{
	D3D11_RENDER_TARGET_VIEW_DESC	RenderTargetViewDesc;
	// Setup the description of the render target view.
	RenderTargetViewDesc.Format = Format;
	RenderTargetViewDesc.ViewDimension = D3D11_RTV_DIMENSION_TEXTURE2D;
	RenderTargetViewDesc.Texture2D.MipSlice = 0;

	// Create the render target view.
	HRESULT hr = pDevice->CreateRenderTargetView(m_pTexture.Get(), &RenderTargetViewDesc, &m_pRenderTargetView);
	DirectX::ThrowIfFailed(hr);

	m_Viewport = { 0.0f, 0.0f, (float) m_TextureDescription.Width, (float) m_TextureDescription.Height, 0.0f, 1.0f };

}

RenderTargetTexture2D::RenderTargetTexture2D(ID3D11Texture2D* pTexture, ID3D11RenderTargetView* pRenderTargetView, ID3D11ShaderResourceView* pShaderResouceView, const D3D11_VIEWPORT *pViewport)
	: Texture2D(pTexture, pShaderResouceView)
{
	assert(pRenderTargetView);
	m_pRenderTargetView = pRenderTargetView;
	if (pViewport == nullptr)
		m_Viewport = { 0.0f, 0.0f, (float) m_TextureDescription.Width, (float) m_TextureDescription.Height, 0.0f, 1.0f };
	else
		m_Viewport = *pViewport;
}

RenderTargetTexture2D::RenderTargetTexture2D()
{

}

RenderTargetTexture2D::RenderTargetTexture2D(RenderTargetTexture2D &&source)
	: Texture2D(std::move(source)),
	m_pRenderTargetView(std::move(source.m_pRenderTargetView)),
	m_Viewport(source.m_Viewport)
{

}

RenderTargetTexture2D& RenderTargetTexture2D::operator=(RenderTargetTexture2D &&source)
{
	Texture2D::operator=(std::move(source));
	m_pRenderTargetView = std::move(source.m_pRenderTargetView);
	m_Viewport = source.m_Viewport;
	return *this;
}

DepthStencilBuffer::DepthStencilBuffer()
{}
DepthStencilBuffer::DepthStencilBuffer(DepthStencilBuffer&&source)
	: Texture2D(std::move(source)),
	m_pDepthStencilView(std::move(source.m_pDepthStencilView))
{
}
DepthStencilBuffer& DepthStencilBuffer::operator = (DepthStencilBuffer&&source)
{
	Texture2D::operator=(std::move(source));
	m_pDepthStencilView = std::move(source.m_pDepthStencilView);
	return *this;
}



DepthStencilBuffer::DepthStencilBuffer(ID3D11Device* pDevice, unsigned int Width, unsigned int Height, _In_opt_ DXGI_FORMAT Format)
	: Texture2D(pDevice, Width, Height, 1, Format, D3D11_USAGE_DEFAULT, D3D11_BIND_DEPTH_STENCIL, 0)
{
	// Initialize the depth stencil view.
	D3D11_DEPTH_STENCIL_VIEW_DESC ViewDesc;
	ZeroMemory(&ViewDesc, sizeof(ViewDesc));
	// Set up the depth stencil view description.
	ViewDesc.Format = Format;
	ViewDesc.ViewDimension = D3D11_DSV_DIMENSION_TEXTURE2D;
	ViewDesc.Texture2D.MipSlice = 0;

	auto pTexture = m_pTexture.Get();
	// Create the depth stencil view.
	HRESULT hr = pDevice->CreateDepthStencilView(pTexture, &ViewDesc, m_pDepthStencilView.GetAddressOf());
	DirectX::ThrowIfFailed(hr);
}

ITexture::ITexture()
{}

ITexture::~ITexture()
{}

Texture2D::Texture2D()
{}
Texture2D::~Texture2D()
{}

DynamicTexture2D::~DynamicTexture2D()
{}

RenderTargetTexture2D::~RenderTargetTexture2D()
{}

DepthStencilBuffer::~DepthStencilBuffer()
{}

std::unique_ptr<ITexture> ITexture::CreateFromDDSFile(_In_ ID3D11Device* pDevice, _In_z_ const wchar_t* szFileName,
	_In_opt_ size_t maxsize,
	_In_opt_ D3D11_USAGE usage,
	_In_opt_ unsigned int bindFlags,
	_In_opt_ unsigned int cpuAccessFlags,
	_In_opt_ unsigned int miscFlags,
	_In_opt_ bool forceSRGB
	)
{
	HRESULT hr;
	wstring exName(szFileName);
	exName = exName.substr(exName.find_last_of(L'.') + 1);
	std::transform(exName.begin(), exName.end(), exName.begin(), ::towupper);

	ID3D11Resource* pResource;
	ID3D11ShaderResourceView *pView;
	if (exName == L"DDS")
	{
		hr = DirectX::CreateDDSTextureFromFileEx(pDevice, szFileName, maxsize, usage, bindFlags, cpuAccessFlags, miscFlags, forceSRGB, &pResource, &pView);
		D3D11_RESOURCE_DIMENSION dimension;
		pResource->GetType(&dimension);

		switch (dimension)
		{
		case D3D11_RESOURCE_DIMENSION_TEXTURE1D:
			break;
		case D3D11_RESOURCE_DIMENSION_TEXTURE2D:
		{
			Texture2D* pTexture2D = new Texture2D(pResource, pView);
			pResource->Release();
			pView->Release();
			return unique_ptr<ITexture>(pTexture2D);
		}

			break;
		case D3D11_RESOURCE_DIMENSION_TEXTURE3D:
			break;
		case D3D11_RESOURCE_DIMENSION_BUFFER:
		case D3D11_RESOURCE_DIMENSION_UNKNOWN:
		default:
			break;
		}
	}
	return nullptr;
}

void ITexture::SaveAsDDSFile(_In_ ID3D11DeviceContext *pDeviceContext, _In_z_ const wchar_t* szFileName)
{
	HRESULT hr = SaveDDSTextureToFile(pDeviceContext, m_pTexture.Get(), szFileName);
	ThrowIfFailed(hr);
}

std::unique_ptr<Texture2D> Texture2D::CreateFromWICFile(_In_ ID3D11Device* pDevice,
	_In_ ID3D11DeviceContext* pDeviceContext,
	_In_z_ const wchar_t* szFileName,
	_In_opt_ size_t maxsize,
	_In_opt_ D3D11_USAGE usage,
	_In_opt_ unsigned int bindFlags,
	_In_opt_ unsigned int cpuAccessFlags,
	_In_opt_ unsigned int miscFlags,
	_In_opt_ bool forceSRGB
	)
{
	HRESULT hr;
	wstring exName(szFileName);
	exName = exName.substr(exName.find_last_of(L'.') + 1);
	std::transform(exName.begin(), exName.end(), exName.begin(), ::towupper);
	std::unique_ptr<Texture2D> pTexture;
	if (exName == L"DDS")
	{
		hr = DirectX::CreateDDSTextureFromFileEx(pDevice, szFileName, maxsize, usage, bindFlags, cpuAccessFlags, miscFlags, forceSRGB, &pTexture->m_pTexture, &pTexture->m_pShaderResourceView);
		ThrowIfFailed(hr);
		D3D11_RESOURCE_DIMENSION dimension;
		pTexture->m_pTexture->GetType(&dimension);
		if (dimension == D3D11_RESOURCE_DIMENSION::D3D11_RESOURCE_DIMENSION_TEXTURE2D)
		{
			ID3D11Texture2D* pTexInterface;
			pTexture->Resource()->QueryInterface<ID3D11Texture2D>(&pTexInterface);
			pTexInterface->GetDesc(&pTexture->m_TextureDescription);
		}
		else
		{
			throw new exception("Unsupported Format");
		}
	}
	else
	{
		hr = DirectX::CreateWICTextureFromFileEx(pDevice, pDeviceContext, szFileName, maxsize, usage, bindFlags, cpuAccessFlags, miscFlags, forceSRGB, &pTexture->m_pTexture, &pTexture->m_pShaderResourceView);
		ThrowIfFailed(hr);
		ID3D11Texture2D* pTexInterface;
		pTexture->Resource()->QueryInterface<ID3D11Texture2D>(&pTexInterface);
		pTexInterface->GetDesc(&pTexture->m_TextureDescription);
		return pTexture;
	}
	return nullptr;
}


std::unique_ptr<Texture2D> Texture2D::CreateFromWICMemory(_In_ ID3D11Device* pDevice,
	_In_ ID3D11DeviceContext* pDeviceContext,
	_In_reads_bytes_(wicDataSize) const uint8_t* wicData,
	_In_ size_t wicDataSize,
	_In_opt_ size_t maxsize,
	_In_opt_ D3D11_USAGE usage,
	_In_opt_ unsigned int bindFlags,
	_In_opt_ unsigned int cpuAccessFlags,
	_In_opt_ unsigned int miscFlags,
	_In_opt_ bool forceSRGB
	)
{
	return nullptr;
}

//--------------------------------------------------------------------------------------
static DXGI_FORMAT EnsureNotTypeless(DXGI_FORMAT fmt)
{
	// Assumes UNORM or FLOAT; doesn't use UINT or SINT
	switch (fmt)
	{
	case DXGI_FORMAT_R32G32B32A32_TYPELESS: return DXGI_FORMAT_R32G32B32A32_FLOAT;
	case DXGI_FORMAT_R32G32B32_TYPELESS:    return DXGI_FORMAT_R32G32B32_FLOAT;
	case DXGI_FORMAT_R16G16B16A16_TYPELESS: return DXGI_FORMAT_R16G16B16A16_UNORM;
	case DXGI_FORMAT_R32G32_TYPELESS:       return DXGI_FORMAT_R32G32_FLOAT;
	case DXGI_FORMAT_R10G10B10A2_TYPELESS:  return DXGI_FORMAT_R10G10B10A2_UNORM;
	case DXGI_FORMAT_R8G8B8A8_TYPELESS:     return DXGI_FORMAT_R8G8B8A8_UNORM;
	case DXGI_FORMAT_R16G16_TYPELESS:       return DXGI_FORMAT_R16G16_UNORM;
	case DXGI_FORMAT_R32_TYPELESS:          return DXGI_FORMAT_R32_FLOAT;
	case DXGI_FORMAT_R8G8_TYPELESS:         return DXGI_FORMAT_R8G8_UNORM;
	case DXGI_FORMAT_R16_TYPELESS:          return DXGI_FORMAT_R16_UNORM;
	case DXGI_FORMAT_R8_TYPELESS:           return DXGI_FORMAT_R8_UNORM;
	case DXGI_FORMAT_BC1_TYPELESS:          return DXGI_FORMAT_BC1_UNORM;
	case DXGI_FORMAT_BC2_TYPELESS:          return DXGI_FORMAT_BC2_UNORM;
	case DXGI_FORMAT_BC3_TYPELESS:          return DXGI_FORMAT_BC3_UNORM;
	case DXGI_FORMAT_BC4_TYPELESS:          return DXGI_FORMAT_BC4_UNORM;
	case DXGI_FORMAT_BC5_TYPELESS:          return DXGI_FORMAT_BC5_UNORM;
	case DXGI_FORMAT_B8G8R8A8_TYPELESS:     return DXGI_FORMAT_B8G8R8A8_UNORM;
	case DXGI_FORMAT_B8G8R8X8_TYPELESS:     return DXGI_FORMAT_B8G8R8X8_UNORM;
	case DXGI_FORMAT_BC7_TYPELESS:          return DXGI_FORMAT_BC7_UNORM;
	default:                                return fmt;
	}
}

StagingTexture2D::StagingTexture2D(ID3D11DeviceContext* pContext, const Texture2D* pSourceTexture, _In_opt_ unsigned int cpuAccessFlags)
{
	if (!pContext || !pSourceTexture)
		throw std::invalid_argument("DeviceContext or Source Texture is null.");

	auto pSource = pSourceTexture->Resource();

	D3D11_RESOURCE_DIMENSION resType = D3D11_RESOURCE_DIMENSION_UNKNOWN;
	pSource->GetType(&resType);

	if (resType != D3D11_RESOURCE_DIMENSION_TEXTURE2D)
		ThrowIfFailed(HRESULT_FROM_WIN32(ERROR_NOT_SUPPORTED));

	ScopedObject<ID3D11Texture2D> pTexture;
	HRESULT hr = pSource->QueryInterface<ID3D11Texture2D>(&pTexture);

	ThrowIfFailed(hr);

	assert(pTexture.Get());

	pTexture->GetDesc(&m_TextureDescription);

	ScopedObject<ID3D11Device> d3dDevice;
	pContext->GetDevice(&d3dDevice);

	if (m_TextureDescription.SampleDesc.Count > 1)
	{
		// MSAA content must be resolved before being copied to a staging texture
		m_TextureDescription.SampleDesc.Count = 1;
		m_TextureDescription.SampleDesc.Quality = 0;

		ScopedObject<ID3D11Texture2D> pTemp;
		hr = d3dDevice->CreateTexture2D(&m_TextureDescription, 0, &pTemp);
		ThrowIfFailed(hr);

		assert(pTemp.Get());

		DXGI_FORMAT fmt = EnsureNotTypeless(m_TextureDescription.Format);

		UINT support = 0;
		hr = d3dDevice->CheckFormatSupport(fmt, &support);
		ThrowIfFailed(hr);

		if (!(support & D3D11_FORMAT_SUPPORT_MULTISAMPLE_RESOLVE))
			throw std::runtime_error("Multisampling resolve failed");

		for (UINT item = 0; item < m_TextureDescription.ArraySize; ++item)
		{
			for (UINT level = 0; level < m_TextureDescription.MipLevels; ++level)
			{
				UINT index = D3D11CalcSubresource(level, item, m_TextureDescription.MipLevels);
				pContext->ResolveSubresource(pTemp.Get(), index, pSource, index, fmt);
			}
		}

		m_TextureDescription.BindFlags = 0;
		m_TextureDescription.MiscFlags &= D3D11_RESOURCE_MISC_TEXTURECUBE;
		m_TextureDescription.CPUAccessFlags = cpuAccessFlags;
		m_TextureDescription.Usage = D3D11_USAGE_STAGING;

		ID3D11Texture2D* pTex;
		hr = d3dDevice->CreateTexture2D(&m_TextureDescription, 0, &pTex);
		ThrowIfFailed(hr);
		assert(m_pTexture.Get());
		pContext->CopyResource(m_pTexture.Get(), pTemp.Get());
		//m_pTexture = pTexture.Get();
	}
	else if ((m_TextureDescription.Usage == D3D11_USAGE_STAGING) && (m_TextureDescription.CPUAccessFlags & cpuAccessFlags))
	{
		// Handle case where the source is already a staging texture we can use directly
		m_pTexture = pTexture.Get();
	}
	else
	{
		// Otherwise, create a staging texture from the non-MSAA source
		m_TextureDescription.BindFlags = 0;
		m_TextureDescription.MiscFlags &= D3D11_RESOURCE_MISC_TEXTURECUBE;
		m_TextureDescription.CPUAccessFlags = cpuAccessFlags;
		m_TextureDescription.Usage = D3D11_USAGE_STAGING;

		ID3D11Texture2D* pTex;
		hr = d3dDevice->CreateTexture2D(&m_TextureDescription, 0, &pTex);
		m_pTexture = pTex;
		ThrowIfFailed(hr);

		assert(m_pTexture.Get());

		pContext->CopyResource(m_pTexture.Get(), pSource);
		//m_pTexture = pTexture.Get();
	}
}

//--------------------------------------------------------------------------------------
// Get surface information for a particular format
//--------------------------------------------------------------------------------------
static void GetSurfaceInfo(_In_ size_t width,
	_In_ size_t height,
	_In_ DXGI_FORMAT fmt,
	_Out_opt_ size_t* outNumBytes,
	_Out_opt_ size_t* outRowBytes,
	_Out_opt_ size_t* outNumRows)
{
	size_t numBytes = 0;
	size_t rowBytes = 0;
	size_t numRows = 0;

	bool bc = false;
	bool packed = false;
	size_t bcnumBytesPerBlock = 0;
	switch (fmt)
	{
	case DXGI_FORMAT_BC1_TYPELESS:
	case DXGI_FORMAT_BC1_UNORM:
	case DXGI_FORMAT_BC1_UNORM_SRGB:
	case DXGI_FORMAT_BC4_TYPELESS:
	case DXGI_FORMAT_BC4_UNORM:
	case DXGI_FORMAT_BC4_SNORM:
		bc = true;
		bcnumBytesPerBlock = 8;
		break;

	case DXGI_FORMAT_BC2_TYPELESS:
	case DXGI_FORMAT_BC2_UNORM:
	case DXGI_FORMAT_BC2_UNORM_SRGB:
	case DXGI_FORMAT_BC3_TYPELESS:
	case DXGI_FORMAT_BC3_UNORM:
	case DXGI_FORMAT_BC3_UNORM_SRGB:
	case DXGI_FORMAT_BC5_TYPELESS:
	case DXGI_FORMAT_BC5_UNORM:
	case DXGI_FORMAT_BC5_SNORM:
	case DXGI_FORMAT_BC6H_TYPELESS:
	case DXGI_FORMAT_BC6H_UF16:
	case DXGI_FORMAT_BC6H_SF16:
	case DXGI_FORMAT_BC7_TYPELESS:
	case DXGI_FORMAT_BC7_UNORM:
	case DXGI_FORMAT_BC7_UNORM_SRGB:
		bc = true;
		bcnumBytesPerBlock = 16;
		break;

	case DXGI_FORMAT_R8G8_B8G8_UNORM:
	case DXGI_FORMAT_G8R8_G8B8_UNORM:
		packed = true;
		break;
	}

	if (bc)
	{
		size_t numBlocksWide = 0;
		if (width > 0)
		{
			numBlocksWide = std::max<size_t>(1, (width + 3) / 4);
		}
		size_t numBlocksHigh = 0;
		if (height > 0)
		{
			numBlocksHigh = std::max<size_t>(1, (height + 3) / 4);
		}
		rowBytes = numBlocksWide * bcnumBytesPerBlock;
		numRows = numBlocksHigh;
	}
	else if (packed)
	{
		rowBytes = ((width + 1) >> 1) * 4;
		numRows = height;
	}
	else
	{
		size_t bpp = DXGIFormatTraits::SizeofDXGIFormatInBits(fmt);
		rowBytes = (width * bpp + 7) / 8; // round up to nearest byte
		numRows = height;
	}

	numBytes = rowBytes * numRows;
	if (outNumBytes)
	{
		*outNumBytes = numBytes;
	}
	if (outRowBytes)
	{
		*outRowBytes = rowBytes;
	}
	if (outNumRows)
	{
		*outNumRows = numRows;
	}
}

std::unique_ptr<uint8_t> StagingTexture2D::GetData(ID3D11DeviceContext *pContext)
{
	size_t rowPitch, slicePitch, rowCount;
	auto & desc = m_TextureDescription;
	GetSurfaceInfo(desc.Width, desc.Height, desc.Format, &slicePitch, &rowPitch, &rowCount);

	// Setup pixels
	std::unique_ptr<uint8_t> pixels(new (std::nothrow) uint8_t[slicePitch]);
	if (!pixels)
		throw std::bad_alloc();

	D3D11_MAPPED_SUBRESOURCE mapped;

	HRESULT hr = pContext->Map(m_pTexture.Get(), 0, D3D11_MAP_READ, 0, &mapped);
	ThrowIfFailed(hr);

	const uint8_t* sptr = reinterpret_cast<const uint8_t*>(mapped.pData);
	if (!sptr)
	{
		pContext->Unmap(m_pTexture.Get(), 0);
		throw std::runtime_error("Resource is invalid.");
	}

	uint8_t* dptr = pixels.get();

	for (size_t h = 0; h < rowCount; ++h)
	{
		size_t msize = std::min<size_t>(rowPitch, mapped.RowPitch);
		memcpy_s(dptr, rowPitch, sptr, msize);
		sptr += mapped.RowPitch;
		dptr += rowPitch;
	}

	pContext->Unmap(m_pTexture.Get(), 0);

	return pixels;
}

StagingTexture2D::StagingTexture2D(ID3D11Texture2D* pTexture)
	: Texture2D(pTexture)
{
}

void DirectX::CubeTexture::Initialize(ID3D11Device * pDevice, const std::wstring(&TextureFiles)[6])
{
	for (int i = 0; i < 6; i++)
	{
		ThrowIfFailed(DirectX::CreateDDSTextureFromFile(pDevice, TextureFiles[i].c_str(), &(m_pTextures[i]), &(m_pTextureView[i])));
	}
}
