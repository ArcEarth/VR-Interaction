#pragma once

#include <string>
#include <d3d11_1.h>
#include <DirectXMath.h>
#include <memory>
#include <functional>
#include <DirectXPackedVector.h>

class FloatingTextWrapper;

class IFloatingText
{
public:
	std::wstring							Content;
	std::wstring							FontFamily;
	float									FontSize;
	DirectX::PackedVector::XMUBYTEN4		FontColor;
	DirectX::XMFLOAT2						Position;
	virtual ~IFloatingText() {}

	void Show()
	{
		visible = true;
	}

	void SetColor(DirectX::FXMVECTOR Color)
	{
		DirectX::PackedVector::XMStoreUByteN4(&FontColor,Color);
	}


	void Hide()
	{
		visible = false;
	}

	bool Visible() const
	{
		return Alive() && visible;
	}

	virtual	bool Alive() const
	{
		return !Released;
	}

	virtual void Release()
	{
		Released = true;
	}


	virtual void Update()
	{
	}


protected:
	/// <summary>
	/// Initializes a new instance of the <see cref="FloatingText"/> class.
	/// </summary>
	/// <param name="content">The content string.</param>
	/// <param name="X">The Position X.</param>
	/// <param name="Y">The Position Y.</param>
	/// <param name="fontSize">Size of the font.</param>
	/// <param name="fontColor">Color of the font.</param>
	/// <param name="fontFamiliy">The font family.</param>
	IFloatingText(const std::wstring& content,float X,float Y,float fontSize = 48,DirectX::FXMVECTOR fontColor = DirectX::Colors::White,const std::wstring fontFamily = L"Arial");

	IFloatingText()
		: Released(false)
		, visible(false)
	{}

	friend FloatingTextWrapper;

protected:
	bool			Released;
	bool			visible;
};

class FloatingTextWrapper
{
public:
	FloatingTextWrapper(ID3D11Device *pDevice);
	~FloatingTextWrapper();

	void Render(ID3D11DeviceContext *pContext);

	//IFloatingText* CreateFloatingText(const std::wstring& content,float X,float Y,float fontSize = 48,DirectX::FXMVECTOR fontColor = DirectX::Colors::White,const std::wstring fontFamily = L"Arial");
	IFloatingText* CreateCustomizedFloatingText(const std::wstring& content,float X,float Y,std::function<void(IFloatingText*)> CustomlizeFunction,float fontSize = 48,DirectX::FXMVECTOR fontColor = DirectX::Colors::White,const std::wstring fontFamily = L"Arial");
	IFloatingText* CreateTimedFloatingText(const std::wstring& content,float X,float Y,double lifeTime,float fontSize = 48,DirectX::FXMVECTOR fontColor = DirectX::Colors::White,bool Fading = true , bool Shifting = false, const std::wstring fontFamily = L"Arial");
	//IFloatingText* CreateFloatingText(const std::wstring& content,float X,float Y,float fontSize = 48,DirectX::FXMVECTOR fontColor = DirectX::Colors::White, const std::wstring fontFamily = L"Arial");
	IFloatingText* CreateFloatingText(const std::wstring& content=L"",float X=0.0f,float Y=0.0f,float fontSize = 48,DirectX::FXMVECTOR fontColor = DirectX::Colors::White, const std::wstring fontFamily = L"Arial");
	//void ShowFloatingTextThisFrame(const std::wstring &Content,float FontSize,float X,float Y,DirectX::FXMVECTOR FontColor, const std::wstring& FontFamily);

private:
	struct Impl;
	std::unique_ptr<Impl>		m_pImpl;
};
