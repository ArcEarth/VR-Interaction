#include "stdafx.h"
#include "FloatingText.h"
#include "FW1FontWrapper.h"
#include "TIMER.h"
#include <wrl\client.h>

#include <list>
#include <map>
#include <algorithm>
#include "DirectXHelper.h"


using namespace DirectX;
using namespace DirectX::PackedVector;
using namespace std;
using namespace Microsoft::WRL;

static const wstring DefaultFontFamily = L"Arial";


IFloatingText::IFloatingText(const std::wstring& content,float X,float Y,float fontSize/* = 48*/,DirectX::FXMVECTOR fontColor/* = DirectX::Colors::White*/,const wstring fontFamily/* = L"Arial"*/)
	: Content(content)
	, FontSize(fontSize)
	, FontFamily(fontFamily)
	, Position(X,Y)
	, visible(true)
	, Released(false)
{
	DirectX::PackedVector::XMStoreUByteN4(&FontColor,fontColor);
}

class CustomlizedFloatingText
	: public IFloatingText
{
public:
	CustomlizedFloatingText(const std::wstring& content,float X,float Y,std::function<void(IFloatingText*)> CustomlizeFunction, float fontSize = 48,DirectX::FXMVECTOR fontColor = DirectX::Colors::White,const wstring fontFamily = L"Arial")
		: IFloatingText(content,X,Y,fontSize,fontColor,fontFamily)
		, pCustomFunc (CustomlizeFunction) 
	{
	}

	~CustomlizedFloatingText()
	{
		Release();
	}

	virtual void Update()
	{
		if (pCustomFunc)
		{
			pCustomFunc(this);
		}
	}

	virtual void Release()
	{
		pCustomFunc = nullptr;
	}

protected:
	std::function<void(IFloatingText*)> pCustomFunc;
};

class TimedFloatingText
	: public IFloatingText
{
public:
	double			LifeTime;
	bool			Fading;
	bool			Shifting;

protected:
	TIMER			LifeTimer;

public:
	TimedFloatingText(const std::wstring& content,float X,float Y,double lifeTime,float fontSize = 48,DirectX::FXMVECTOR fontColor = DirectX::Colors::White,bool fading = true , bool shifting = false, const wstring fontFamily = L"Arial")
		: LifeTime(lifeTime)
		, Fading(fading)
		, Shifting(shifting)
		, IFloatingText(content,X,Y,fontSize,fontColor,fontFamily)
	{
		DirectX::PackedVector::XMStoreUByteN4(&FontColor,fontColor);
		LifeTimer.Reset();
	}

	virtual void Update()
	{
		float alpha = this->RemainLifeTimeRatio();
		if (Fading)
			this->FontColor.w = static_cast<uint8_t>(alpha * 255);
		if (Shifting)
			this->Position.y -= this->FontSize * (1.0f-alpha);
	}

	virtual bool Alive() const
	{
		return !Released && LifeTimer.GetTime() < LifeTime;
	}

	float RemainLifeTimeRatio() const
	{
		return 1.0f - (float)(LifeTimer.GetTime() / LifeTime);
	}
};

class TemporaryFloatingText
	: public IFloatingText
{
public:
	TemporaryFloatingText(const std::wstring& content,float X,float Y,unsigned int LifeFrameCount = 1, float fontSize = 48,DirectX::FXMVECTOR fontColor = DirectX::Colors::White,const wstring fontFamily = L"Arial")
		: IFloatingText(content,X,Y,fontSize,fontColor,fontFamily)
		, Life(LifeFrameCount)
	{
	}

	virtual void Update()
	{
		--Life;
	}

	virtual bool Alive() const
	{
		return !Released && Life > 0;
	}


protected:
	unsigned int Life;
};



struct FloatingTextWrapper::Impl	
{
public:
	Impl(ID3D11Device *pDevice)
	{
		HRESULT hr;
		hr = FW1CreateFactory(FW1_VERSION,&m_pFWFactory);
		ThrowIfFailed(hr);
		hr = m_pFWFactory->CreateFontWrapper(pDevice,DefaultFontFamily.data(),&m_pWrapper);
		ThrowIfFailed(hr);
	};
	~Impl()
	{
	};

	void Render(ID3D11DeviceContext *pContext)
	{
		for (auto& text : TextList)
		{
			if (text->Visible())
			{
				//XMUBYTEN4 color = text->FontColor;
				//float alpha = text->RemainLifeTimeRatio();
				////color.v = 0xffffffff;
				//color.w = static_cast<uint8_t>(alpha * 255);
				//XMFLOAT2 position = text->Position;
				//position.y += text->FontSize * (1.0f-alpha);
				text->Update();
				m_pWrapper->DrawString(pContext,text->Content.data(),text->FontFamily.data(),text->FontSize,text->Position.x,text->Position.y,text->FontColor.v,FW1_VCENTER|FW1_CENTER);
			}
		}

		pContext->GSSetShader(nullptr,nullptr,0);

		TextList.remove_if([](const std::unique_ptr<IFloatingText> &pText)->bool
		{
			return !pText->Alive();
		});
	}

	ComPtr<IFW1FontWrapper>					m_pWrapper;
	ComPtr<IFW1Factory>						m_pFWFactory;
	//std::unique_ptr<RectDrawer>				m_pRectDrawer;

	list<std::unique_ptr<IFloatingText>> TextList;
};


FloatingTextWrapper::FloatingTextWrapper(ID3D11Device *pDevice)
	: m_pImpl(new Impl(pDevice))
{
}

FloatingTextWrapper::~FloatingTextWrapper()
{
}

IFloatingText* FloatingTextWrapper::CreateCustomizedFloatingText(const std::wstring& content,float X,float Y,std::function<void(IFloatingText*)> CustomlizeFunction,float fontSize,DirectX::FXMVECTOR fontColor,const std::wstring fontFamily)
{
	m_pImpl->TextList.emplace_back(new CustomlizedFloatingText(content,X,Y,CustomlizeFunction,fontSize,fontColor,DefaultFontFamily));
	return m_pImpl->TextList.back().get();
}

IFloatingText* FloatingTextWrapper::CreateTimedFloatingText(const std::wstring& content,float X,float Y,double lifeTime,float fontSize,DirectX::FXMVECTOR fontColor,bool Fading, bool Shifting, const std::wstring fontFamily)
{
	m_pImpl->TextList.emplace_back(new TimedFloatingText(content,X,Y,lifeTime,fontSize,fontColor,Fading,Shifting,DefaultFontFamily));
	return m_pImpl->TextList.back().get();
}

//IFloatingText* FloatingTextWrapper::CreateFloatingText(const std::wstring& content,float X,float Y, float fontSize,DirectX::FXMVECTOR fontColor,const std::wstring fontFamily)
//{
//	m_pImpl->TextList.emplace_back(new TemporaryFloatingText(content,X,Y,1,fontSize,fontColor,DefaultFontFamily));
//	return m_pImpl->TextList.back().get();
//}

IFloatingText* FloatingTextWrapper::CreateFloatingText(const std::wstring& content , float X ,float Y,float fontSize,DirectX::FXMVECTOR fontColor, const std::wstring fontFamily)
{
	m_pImpl->TextList.emplace_back(new IFloatingText(content,X,Y,fontSize,fontColor,fontFamily));
	return m_pImpl->TextList.back().get();
}



void FloatingTextWrapper::Render(ID3D11DeviceContext *pContext)
{
	m_pImpl->Render(pContext);
}



