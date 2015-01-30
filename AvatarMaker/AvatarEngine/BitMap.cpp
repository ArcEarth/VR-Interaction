#include "stdafx.h"
#include "BitMap.h"
#include "MathHelper.h"
#include <memory>
#include <exception>
#include "DXGIFormatHelper.h"

using namespace DirectX;
using namespace DirectX::PackedVector;
using namespace DXGIFormatTraits;

Bitmap::Bitmap(void)
	: m_Width(0) , m_Height(0) , m_RawData(nullptr) , m_Format(DXGI_FORMAT::DXGI_FORMAT_UNKNOWN)
{
}

void Bitmap::Initalize(DXGI_FORMAT PixelFormat,unsigned width,unsigned height) //throw(std::bad_alloc)
{
	Release();
	m_Width = width;
	m_Height = height;
	m_Format = PixelFormat;
	auto size_pixel = SizeofDXGIFormatInBytes(PixelFormat);
	m_RawData = malloc(m_Width*m_Height*size_pixel);
	if (m_RawData == nullptr)
		throw std::bad_alloc();
}

Bitmap::Bitmap(unsigned width , unsigned height , DXGI_FORMAT PixelFormat , FXMVECTOR Background )
	: m_Width(0) , m_Height(0) , m_RawData(nullptr) , m_Format(DXGI_FORMAT::DXGI_FORMAT_UNKNOWN)
{
	Initalize(PixelFormat,width,height);
	Dye(Background);
}

Bitmap::Bitmap(unsigned width , unsigned height , DXGI_FORMAT PixelFormat , const void* pRawData , size_t Stride )
	: m_Width(width) , m_Height(height) , m_RawData(nullptr) , m_Format(DXGI_FORMAT::DXGI_FORMAT_UNKNOWN)
{
	Initalize(PixelFormat,width,height);
	Fill(pRawData,Stride);
}

void Bitmap::Fill(const void* pRawData , size_t Stride)
{
	memcpy(m_RawData,pRawData,SizeInBytes());
}

void Bitmap::FromVectorStream(const XMFLOAT4* pRawData)
{
	auto size = m_Width * m_Height;
	size_t stride = SizeofDXGIFormatInBytes(m_Format);
	for (size_t i = 0; i < size; i++)
	{
		XMVECTOR vPixel = XMLoadFloat4(pRawData + i);
		void* pDst = (char*)m_RawData + stride*i;
		XMStoreDXGIFormat(pDst,m_Format,vPixel);
	}
}

void Bitmap::Resize(unsigned width , unsigned height , DXGI_FORMAT NewFormat)
{
	if (NewFormat==0) 
		NewFormat = m_Format;
	if ((!m_RawData || SizeInBytes() < width*height*SizeofDXGIFormatInBytes(NewFormat)))
	{
		Initalize(NewFormat,width,height);
	} else
	{
		m_Width = width;
		m_Height = height;
	}
}

Bitmap::Bitmap(const Bitmap& rhs)
	: m_Width(0) , m_Height(0) , m_RawData(nullptr)
{
	*this = rhs;
}

Bitmap& Bitmap::operator= (const Bitmap& rhs)
{

	Resize(rhs.Width(),rhs.Height(),rhs.Format());
	Fill(rhs.m_RawData);
	return *this;
}

Bitmap::Bitmap (Bitmap&& rhs)
{
	m_RawData = nullptr;
	m_Width = 0;
	m_Height = 0;
	*this = std::move(rhs);
}

Bitmap& Bitmap::operator= (Bitmap&& rhs)
{
	Release();
	m_Height = rhs.m_Height;
	m_Width = rhs.m_Width;
	m_RawData = rhs.m_RawData;
	m_Format = rhs.m_Format;

	rhs.m_RawData = nullptr;
	return *this;
}

void Bitmap::Release()
{
	if(m_RawData)
	{
		free(m_RawData);
		m_RawData = nullptr;
	}
	m_Height = 0;
	m_Width = 0;
}

Bitmap::~Bitmap(void)
{
	Release();
}


size_t Bitmap::SizeInBytes() const
{
	return m_Width*m_Height*SizeofDXGIFormatInBytes(Format());
}

void* Bitmap::GetPixelAddress(unsigned int x,unsigned int y) const
{
#ifdef _DEBUG
	if (x>=m_Width || y>=m_Height) throw std::out_of_range("Index overflow");
#endif
	x += y*m_Width;
	x *= DXGIFormatTraits::SizeofDXGIFormatInBytes(m_Format);
	return (char*)m_RawData + x;
}

void* Bitmap::GetPixelAddress(float u,float v) const
{
#ifdef _DEBUG
	if (u<0.0 || u>1.0 || v<0.0 || v>1.0) 
		throw std::out_of_range("UV Coordinate overflow.");
#endif
	auto x = (size_t)(u*(m_Width-1));
	auto y = (size_t)(v*(m_Height-1));
	x += y*m_Width;
	x *= DXGIFormatTraits::SizeofDXGIFormatInBytes(m_Format);
	return (char*)m_RawData + x;
}

XMVECTOR Bitmap::GetPixel(unsigned x,unsigned y) const
{
	void* pPixel = GetPixelAddress(x,y);
	return XMLoadDXGIFormat(pPixel,m_Format);
}

XMVECTOR Bitmap::GetPixel(float u,float v) const
{
	void* pPixel = GetPixelAddress(u,v);
	return XMLoadDXGIFormat(pPixel,m_Format);
}


void Bitmap::SetPixel(unsigned x , unsigned y , FXMVECTOR PixelColor)
{
	void* pPixel = GetPixelAddress(x,y);
	XMStoreDXGIFormat(pPixel,m_Format,PixelColor);
}

void Bitmap::SetPixel(float u , float v , FXMVECTOR PixelColor)
{
	void* pPixel = GetPixelAddress(u,v);
	XMStoreDXGIFormat(pPixel,m_Format,PixelColor);
}

void Bitmap::Dye(FXMVECTOR Color){
	if (SizeInBytes() == 0)
		return;
	//std::cout<<"Dye was called. SizeInByte == " << SizeInBytes() << std::endl;
	auto elementSize = DXGIFormatTraits::SizeofDXGIFormatInBytes(m_Format);
	auto pixelCount = m_Width*m_Height;
	XMStoreDXGIFormat(m_RawData,m_Format,Color);
	uint8_t* pData = reinterpret_cast<uint8_t*>(m_RawData);
	pData += elementSize;
	unsigned int blockLength = 1;
	while (blockLength*2 < pixelCount)
	{
		memcpy(pData,m_RawData,blockLength*elementSize);
		pData += blockLength*elementSize;
		blockLength <<= 1; 
	}

	blockLength = pixelCount - blockLength;
	memcpy(pData,m_RawData,blockLength*elementSize);
}

void ChangeFormat(DXGI_FORMAT NewFormat)
{
	throw;
}