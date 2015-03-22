#include "pch_directX.h"
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

Texture::Texture(Texture&& Src)
	: m_pResource(std::move(Src.m_pResource))
	, m_pShaderResourceView(std::move(Src.ShaderResourceView()))
{
}

Texture& Texture::operator = (Texture&& Src)
{
	m_pResource = std::move(Src.m_pResource);
	m_pShaderResourceView = std::move(Src.ShaderResourceView());
	return *this;
}

Texture2D::Texture2D(Texture2D &&Src)
	:DirectX::Texture(std::move(static_cast<DirectX::Texture&>(Src)))
	, m_pTexture(std::move(Src.m_pTexture))
	, m_Description(Src.m_Description)
{
}

Texture2D& Texture2D::operator = (Texture2D &&Src)
{
	Texture::operator=(std::move(Src));
	m_pTexture = Src.m_pTexture;
	m_Description = Src.m_Description;
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
	if (multisamplesCount > 1)
		bindFlags &= ~D3D11_BIND_SHADER_RESOURCE; // Shader resources is not valiad for MSAA texture
	D3D11_TEXTURE2D_DESC &TextureDesc = m_Description;
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

	HRESULT hr = pDevice->CreateTexture2D(&TextureDesc, NULL, &m_pTexture);
	DirectX::ThrowIfFailed(hr);
	hr = m_pTexture.As<ID3D11Resource>(&m_pResource);
	DirectX::ThrowIfFailed(hr);

	if ((bindFlags & D3D11_BIND_DEPTH_STENCIL) || !(bindFlags & D3D11_BIND_SHADER_RESOURCE) || !(format == DXGI_FORMAT_R32_TYPELESS))
	{
		m_pShaderResourceView = nullptr;
		return;
	}

	CD3D11_SHADER_RESOURCE_VIEW_DESC ShaderResourceViewDesc(m_pTexture.Get(),D3D11_SRV_DIMENSION_TEXTURE2D);/*
	ShaderResourceViewDesc.Format = TextureDesc.Format;
	ShaderResourceViewDesc.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2D;
	ShaderResourceViewDesc.Texture2D.MostDetailedMip = 0;
	ShaderResourceViewDesc.Texture2D.MipLevels = 1;*/

	hr = pDevice->CreateShaderResourceView(m_pResource.Get(), &ShaderResourceViewDesc, &m_pShaderResourceView);
	DirectX::ThrowIfFailed(hr);
}

Texture2D::Texture2D(ID3D11Texture2D* pTexture, ID3D11ShaderResourceView* pResourceView)
{
	assert(pTexture);
	m_pTexture = pTexture;
	HRESULT hr = m_pTexture.As<ID3D11Resource>(&m_pResource);
	DirectX::ThrowIfFailed(hr);

	m_pTexture->GetDesc(&m_Description);
	m_pShaderResourceView = pResourceView;
}

Texture2D::Texture2D(ID3D11ShaderResourceView* pResourceView)
{
	ComPtr<ID3D11Resource> pResource;
	pResourceView->GetResource(&pResource);
	HRESULT hr = pResource.As(&m_pTexture);
	DirectX::ThrowIfFailed(hr);
	m_pTexture->GetDesc(&m_Description);
	m_pShaderResourceView = pResourceView;
	m_pResource = std::move(pResource);
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
	auto hr = pContext->Map(m_pResource.Get(), 0, D3D11_MAP_WRITE_DISCARD, 0, &Resouce);
	DirectX::ThrowIfFailed(hr);

	memcpy(Resouce.pData, Raw_Data, element_size * Width() * Height());
	pContext->Unmap(m_pResource.Get(), 0);
}

DynamicTexture2D::DynamicTexture2D(_In_ ID3D11Device* pDevice, _In_ unsigned int Width, _In_ unsigned int Height, _In_opt_ DXGI_FORMAT Format)
	: Texture2D(pDevice, Width, Height, 1, Format, D3D11_USAGE_DYNAMIC, D3D11_BIND_SHADER_RESOURCE, D3D11_CPU_ACCESS_WRITE)
{
}

RenderTargetTexture2D::RenderTargetTexture2D(_In_ ID3D11Device* pDevice, _In_ unsigned int Width, _In_ unsigned int Height,
	_In_opt_ DXGI_FORMAT Format, _In_opt_ UINT MultiSampleCount , _In_opt_ UINT MultiSampleQuality , _In_opt_ bool Shared)
	: Texture2D(pDevice, Width, Height, 1, Format, D3D11_USAGE_DEFAULT, D3D11_BIND_RENDER_TARGET | (MultiSampleCount == 1 ? D3D11_BIND_SHADER_RESOURCE : 0), 0, Shared ? D3D11_RESOURCE_MISC_SHARED : 0, MultiSampleCount, MultiSampleQuality)
{
	CD3D11_RENDER_TARGET_VIEW_DESC	RenderTargetViewDesc(m_pTexture.Get(), D3D11_RTV_DIMENSION_TEXTURE2D);
	// Setup the description of the render target view.
	//RenderTargetViewDesc.Format = Format;
	//RenderTargetViewDesc.ViewDimension = D3D11_RTV_DIMENSION_TEXTURE2D;
	//RenderTargetViewDesc.Texture2D.MipSlice = 0;

	// Create the render target view.
	HRESULT hr = pDevice->CreateRenderTargetView(m_pResource.Get(), nullptr, &m_pRenderTargetView);
	DirectX::ThrowIfFailed(hr);
}

RenderTargetTexture2D::RenderTargetTexture2D(ID3D11Texture2D* pTexture, ID3D11RenderTargetView* pRenderTargetView, ID3D11ShaderResourceView* pShaderResouceView)
	: Texture2D(pTexture, pShaderResouceView)
{
	assert(pRenderTargetView);
	m_pRenderTargetView = pRenderTargetView;
}

inline DirectX::RenderTargetTexture2D::RenderTargetTexture2D(ID3D11RenderTargetView * pRenderTargetView)
{
	m_pRenderTargetView = pRenderTargetView;
	pRenderTargetView->GetResource(&m_pResource);
	m_pResource.As(&m_pTexture);
	m_pTexture->GetDesc(&m_Description);
}

RenderTargetTexture2D::RenderTargetTexture2D()
{

}

RenderTargetTexture2D::RenderTargetTexture2D(RenderTargetTexture2D &&source)
	: Texture2D(std::move(source)),
	m_pRenderTargetView(std::move(source.m_pRenderTargetView))
{

}

RenderTargetTexture2D& RenderTargetTexture2D::operator=(RenderTargetTexture2D &&source)
{
	Texture2D::operator=(std::move(source));
	m_pRenderTargetView = std::move(source.m_pRenderTargetView);
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



DirectX::DepthStencilBuffer::DepthStencilBuffer(ID3D11Texture2D * pTexture, ID3D11DepthStencilView * pDSV)
	: Texture2D(pTexture,nullptr)
{
	m_pDepthStencilView = pDSV;
}

DirectX::DepthStencilBuffer::DepthStencilBuffer(ID3D11DepthStencilView * pDSV)
{
	m_pDepthStencilView = pDSV;
	m_pDepthStencilView->GetResource(&m_pResource);
	m_pResource.As(&m_pTexture);
	m_pTexture->GetDesc(&m_Description);
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

	auto pTexture = m_pResource.Get();
	// Create the depth stencil view.
	HRESULT hr = pDevice->CreateDepthStencilView(pTexture, &ViewDesc, m_pDepthStencilView.GetAddressOf());
	DirectX::ThrowIfFailed(hr);
}

Texture::Texture()
{}

DirectX::Texture::Texture(ID3D11Resource * pResource, ID3D11ShaderResourceView * pResourceView)
{
	m_pResource = pResource;
	m_pShaderResourceView = pResourceView;
}

Texture::~Texture()
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

Texture Texture::CreateFromDDSFile(_In_ ID3D11Device* pDevice, _In_z_ const wchar_t* szFileName,
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

		// It's an cube texture
		if (miscFlags & D3D11_RESOURCE_MISC_TEXTURECUBE)
		{
			CubeTexture texture(pResource, pView);
			return texture;
		}

		D3D11_RESOURCE_DIMENSION dimension;
		pResource->GetType(&dimension);

		switch (dimension)
		{
		case D3D11_RESOURCE_DIMENSION_TEXTURE1D:
			break;
		case D3D11_RESOURCE_DIMENSION_TEXTURE2D:
		{
			Texture2D texture2D (pView);
			pResource->Release();
			pView->Release();
			return texture2D;
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

void Texture::SaveAsDDSFile(_In_ ID3D11DeviceContext *pDeviceContext, _In_z_ const wchar_t* szFileName)
{
	HRESULT hr = SaveDDSTextureToFile(pDeviceContext, m_pResource.Get(), szFileName);
	ThrowIfFailed(hr);
}

Texture2D Texture2D::CreateFromWICFile(_In_ ID3D11Device* pDevice,
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
	Texture2D texture;
	if (exName == L"DDS")
	{
		hr = DirectX::CreateDDSTextureFromFileEx(pDevice, szFileName, maxsize, usage, bindFlags, cpuAccessFlags, miscFlags, forceSRGB, &texture.m_pResource, &texture.m_pShaderResourceView);
		ThrowIfFailed(hr);
		D3D11_RESOURCE_DIMENSION dimension;
		texture.m_pResource->GetType(&dimension);
		if (dimension == D3D11_RESOURCE_DIMENSION::D3D11_RESOURCE_DIMENSION_TEXTURE2D)
		{
			ID3D11Texture2D* pTexInterface;
			texture.Resource()->QueryInterface<ID3D11Texture2D>(&pTexInterface);
			pTexInterface->GetDesc(&texture.m_Description);
		}
		else
		{
			throw new exception("Unsupported Format");
		}
	}
	else
	{
		hr = DirectX::CreateWICTextureFromFileEx(pDevice, pDeviceContext, szFileName, maxsize, usage, bindFlags, cpuAccessFlags, miscFlags, forceSRGB, &texture.m_pResource, &texture.m_pShaderResourceView);
		ThrowIfFailed(hr);
		ID3D11Texture2D* pTexInterface;
		texture.Resource()->QueryInterface<ID3D11Texture2D>(&pTexInterface);
		pTexInterface->GetDesc(&texture.m_Description);
		return texture;
	}
	return texture;
}


Texture2D Texture2D::CreateFromWICMemory(_In_ ID3D11Device* pDevice,
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
	return Texture2D();
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

	pTexture->GetDesc(&m_Description);

	ScopedObject<ID3D11Device> d3dDevice;
	pContext->GetDevice(&d3dDevice);

	if (m_Description.SampleDesc.Count > 1)
	{
		// MSAA content must be resolved before being copied to a staging texture
		m_Description.SampleDesc.Count = 1;
		m_Description.SampleDesc.Quality = 0;

		ScopedObject<ID3D11Texture2D> pTemp;
		hr = d3dDevice->CreateTexture2D(&m_Description, 0, &pTemp);
		ThrowIfFailed(hr);

		assert(pTemp.Get());

		DXGI_FORMAT fmt = EnsureNotTypeless(m_Description.Format);

		UINT support = 0;
		hr = d3dDevice->CheckFormatSupport(fmt, &support);
		ThrowIfFailed(hr);

		if (!(support & D3D11_FORMAT_SUPPORT_MULTISAMPLE_RESOLVE))
			throw std::runtime_error("Multisampling resolve failed");

		for (UINT item = 0; item < m_Description.ArraySize; ++item)
		{
			for (UINT level = 0; level < m_Description.MipLevels; ++level)
			{
				UINT index = D3D11CalcSubresource(level, item, m_Description.MipLevels);
				pContext->ResolveSubresource(pTemp.Get(), index, pSource, index, fmt);
			}
		}

		m_Description.BindFlags = 0;
		m_Description.MiscFlags &= D3D11_RESOURCE_MISC_TEXTURECUBE;
		m_Description.CPUAccessFlags = cpuAccessFlags;
		m_Description.Usage = D3D11_USAGE_STAGING;

		ID3D11Texture2D* pTex;
		hr = d3dDevice->CreateTexture2D(&m_Description, 0, &pTex);
		ThrowIfFailed(hr);
		assert(m_pResource.Get());
		pContext->CopyResource(m_pResource.Get(), pTemp.Get());
		//m_pTexture = pTexture.Get();
	}
	else if ((m_Description.Usage == D3D11_USAGE_STAGING) && (m_Description.CPUAccessFlags & cpuAccessFlags))
	{
		// Handle case where the source is already a staging texture we can use directly
		m_pResource = pTexture.Get();
	}
	else
	{
		// Otherwise, create a staging texture from the non-MSAA source
		m_Description.BindFlags = 0;
		m_Description.MiscFlags &= D3D11_RESOURCE_MISC_TEXTURECUBE;
		m_Description.CPUAccessFlags = cpuAccessFlags;
		m_Description.Usage = D3D11_USAGE_STAGING;

		ID3D11Texture2D* pTex;
		hr = d3dDevice->CreateTexture2D(&m_Description, 0, &pTex);
		m_pResource = pTex;
		ThrowIfFailed(hr);

		assert(m_pResource.Get());

		pContext->CopyResource(m_pResource.Get(), pSource);
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
	auto & desc = m_Description;
	GetSurfaceInfo(desc.Width, desc.Height, desc.Format, &slicePitch, &rowPitch, &rowCount);

	// Setup pixels
	std::unique_ptr<uint8_t> pixels(new (std::nothrow) uint8_t[slicePitch]);
	if (!pixels)
		throw std::bad_alloc();

	D3D11_MAPPED_SUBRESOURCE mapped;

	HRESULT hr = pContext->Map(m_pResource.Get(), 0, D3D11_MAP_READ, 0, &mapped);
	ThrowIfFailed(hr);

	const uint8_t* sptr = reinterpret_cast<const uint8_t*>(mapped.pData);
	if (!sptr)
	{
		pContext->Unmap(m_pResource.Get(), 0);
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

	pContext->Unmap(m_pResource.Get(), 0);

	return pixels;
}

StagingTexture2D::StagingTexture2D(ID3D11Texture2D* pTexture)
	: Texture2D(pTexture)
{
}

DirectX::RenderTarget::RenderTarget() {}

DirectX::RenderTarget::RenderTarget(RenderTargetTexture2D & colorBuffer, DepthStencilBuffer & dsBuffer, const D3D11_VIEWPORT & viewPort)
	: m_ColorBuffer(colorBuffer), m_DepthStencilBuffer(dsBuffer), m_Viewport(viewPort)
{}

DirectX::RenderTarget::RenderTarget(ID3D11Device * pDevice, size_t width, size_t height)
	: m_ColorBuffer(pDevice, width, height), m_DepthStencilBuffer(pDevice, width, height)
{
	m_Viewport.TopLeftX = 0;
	m_Viewport.TopLeftY = 0;
	m_Viewport.Width = (float) width;
	m_Viewport.Height = (float) height;
	m_Viewport.MinDepth = D3D11_MIN_DEPTH;
	m_Viewport.MaxDepth = D3D11_MAX_DEPTH;
}

RenderTarget DirectX::RenderTarget::Subview(const D3D11_VIEWPORT * pViewport)
{
	RenderTarget target(m_ColorBuffer, m_DepthStencilBuffer, *pViewport);
	return target;
}

/// <summary>
/// return a Const reference to the ViewPort binding this render target texture for RS stage.
/// </summary>
/// <returns> the return value is default to be (0,0) at left-top corner , while min-max depth is (0,1000) </returns>

const D3D11_VIEWPORT & DirectX::RenderTarget::ViewPort() const
{
	return m_Viewport;
}

void DirectX::RenderTarget::Clear(ID3D11DeviceContext * pContext, FXMVECTOR Color)
{
	m_ColorBuffer.Clear(pContext, Color);
	m_DepthStencilBuffer.Clear(pContext);
}

/// <summary>
/// return a Reference to the ViewPort binding this render target texture for RS stage.
/// </summary>
/// <returns> the return value is default to be (0,0) at left-top corner , while min-max depth is (0,1000) </returns>

D3D11_VIEWPORT & DirectX::RenderTarget::ViewPort()
{
	return m_Viewport;
}

RenderTargetTexture2D & DirectX::RenderTarget::ColorBuffer()
{
	return m_ColorBuffer;
}

const RenderTargetTexture2D & DirectX::RenderTarget::ColorBuffer() const
{
	return m_ColorBuffer;
}

DepthStencilBuffer & DirectX::RenderTarget::DepthBuffer()
{
	return m_DepthStencilBuffer;
}

const DepthStencilBuffer & DirectX::RenderTarget::DepthBuffer() const
{
	return m_DepthStencilBuffer;
}

/// <summary>
/// Sets as render target with default no DepthStencil.
/// </summary>
/// <param name="pDeviceContext">The pointer to device context.</param>
/// <param name="pDepthStencil">The pointer to depth stencil view.</param>

void DirectX::RenderTarget::SetAsRenderTarget(ID3D11DeviceContext * pDeviceContext)
{
	auto pTargetView = m_ColorBuffer.RenderTargetView();
	pDeviceContext->RSSetViewports(1, &m_Viewport);
	pDeviceContext->OMSetRenderTargets(1, &pTargetView, m_DepthStencilBuffer);
}

DirectX::CubeTexture::CubeTexture(ID3D11Resource * pResource, ID3D11ShaderResourceView * pResourceView)
	: Texture(pResource, pResourceView)
{
}

void DirectX::CubeTexture::Initialize(ID3D11Device * pDevice, const std::wstring(&TextureFiles)[6])
{
	for (int i = 0; i < 6; i++)
	{
		DirectX::ThrowIfFailed(DirectX::CreateDDSTextureFromFile(pDevice, TextureFiles[i].c_str(), &(m_pTextures[i]), &(m_pTextureView[i])));
	}
}

DirectX::CubeTexture::CubeTexture(ID3D11Device * pDevice, const std::wstring(&TextureFiles)[6])
{
	Initialize(pDevice, TextureFiles);
}

DirectX::CubeTexture::CubeTexture()
{}

DirectX::CubeTexture::~CubeTexture()
{}

ID3D11ShaderResourceView * DirectX::CubeTexture::at(unsigned int face)
{
	return m_pTextureView[face];
}

ID3D11ShaderResourceView * const * DirectX::CubeTexture::ResourcesView()
{
	return m_pTextureView;
}
