#include <DirectXMath.h>
#include <DirectXColors.h>

#pragma once
#define ALPHA_DEFAULT 1.0f;
namespace DirectX
{

class Bitmap
{
public:
	//typedef PixelType;

	Bitmap(void);
	Bitmap(unsigned width , unsigned height , DXGI_FORMAT PixelFormat = DXGI_FORMAT_R8G8B8A8_UNORM , FXMVECTOR Background = Colors::White);
	Bitmap(unsigned width , unsigned height , DXGI_FORMAT PixelFormat , const void* pRawData , size_t Stride = 0);
	Bitmap(const Bitmap& rhs);
	Bitmap(Bitmap&& rhs);

	~Bitmap(void);

	void Release();
	void Initalize(DXGI_FORMAT PixelFormat,unsigned width,unsigned height);
	void Fill(const void* pRawData , size_t Stride = 0);
	void FromVectorStream(const XMFLOAT4* pRawData);
	void Resize(unsigned width,unsigned height , DXGI_FORMAT NewFormat = DXGI_FORMAT_UNKNOWN);
	void ChangeFormat(DXGI_FORMAT NewFormat);

	Bitmap& operator= (const Bitmap& rhs);
	Bitmap& operator= (Bitmap&& rhs);

	// Accesser
	//inline Pixel& operator()(unsigned int x, unsigned int y) {return at(x,y);}
	//inline const Pixel& operator()(unsigned int x, unsigned int y) const {return at(x,y);}
	//inline Pixel& operator()(float u,float v) {return at(u,v);}
	//inline const Pixel& operator()(float u,float v) const {return at(u,v);}

	//inline Pixel& at(float u,float v);
	//inline Pixel& at(unsigned x,unsigned y);
	//inline const Pixel& at(float u,float v) const;
	//inline const Pixel& at(unsigned x,unsigned y) const;

	XMVECTOR GetPixel(unsigned x,unsigned y) const;
	void SetPixel(unsigned x , unsigned y , FXMVECTOR PixelColor);
	XMVECTOR GetPixel(float u,float v) const;
	void SetPixel(float u , float v , FXMVECTOR PixelColor);

	void Dye(FXMVECTOR Color);

	inline DXGI_FORMAT Format() const
	{
		return m_Format;
	}

	inline const void* RawData() const
	{
		return m_RawData;
	}

	void* RawData()
	{
		return m_RawData;
	}

	// The Height of this bitmip in pixel
	size_t Height() const
	{
		return m_Height;
	}

	// The Width of this bitmip in pixel
	size_t Width() const
	{
		return m_Width;
	}

	// Returen the size of this bitmap in Bytes.
	size_t SizeInBytes() const;

protected:
	void* GetPixelAddress(unsigned int x,unsigned int y) const;
	void* GetPixelAddress(float u,float v) const;

protected:
	DXGI_FORMAT m_Format;
	size_t		m_Width,m_Height;
	void*		m_RawData;
};

//Bitmap::Pixel& Bitmap::at(float u,float v){
//#ifdef _DEBUG
//	if (u<0.0 || u>1.0 || v<0.0 || v>1.0) throw("Coordinate overflow");
//#endif
//	unsigned x = (unsigned int)(u*(m_Width-1)), y = (unsigned int)(v*(m_Height-1));
//	return m_RawData[x+y*m_Width];
//}
//
//Bitmap::Pixel& Bitmap::at(unsigned x,unsigned y){
//#ifdef _DEBUG
//	if (x>=m_Width || y>=m_Height) throw("Index overflow");
//#endif
//	return m_RawData[x+y*m_Width];
//}
//
//const Bitmap::Pixel& Bitmap::at(float u,float v) const
//{
//#ifdef _DEBUG
//	if (u<0.0 || u>1.0 || v<0.0 || v>1.0) throw("Coordinate overflow");
//#endif
//	unsigned x = (unsigned int)(u*(m_Width-1)), y = (unsigned int)(v*(m_Height-1));
//	return m_RawData[x+y*m_Width];
//}
//
//const Bitmap::Pixel& Bitmap::at(unsigned x,unsigned y) const
//{
//#ifdef _DEBUG
//	if (x>=m_Width || y>=m_Height) throw("Index overflow");
//#endif
//	return m_RawData[x+y*m_Width];
//}


}