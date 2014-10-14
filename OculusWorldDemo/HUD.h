#pragma once
#include <OVR_Kernel.h>
#include "../CommonSrc/Render/Render_Device.h"
#include <functional>
#include <mutex>
//#include <SimpleMath.h>

using OVR::Ptr;
using OVR::RefCountBase;
using OVR::Array;

enum class HorizentalAlignmentEnum : unsigned
{
	Left,
	Center,
	Right,
	Stretch,
};

enum class VerticalAlignmentEnum : unsigned
{
	Top,
	Center,
	Bottom,
	Stretch,
};

template <class T>
class Property
{
public:
	operator const T&() const
	{
		return get();
	}

	void operator=(const T& rhs)
	{
		set(rhs);
	}

	std::function<const T&(void)> get;
	std::function<void(const T& value)> set;
};

template <class T>
class AutoProperty : public Property < T >
{
	AutoProperty()
	{
		get = &DefaultGet;
		set = &DefaultSet;
	}
protected:
	T		entity;
	const T& DefaultGet() const
	{
		return entity;
	}

	void DefaultSet(const T& rhs)
	{
		entity = rhs;
	}
};

struct VisualBase;
typedef OVR::Ptr<VisualBase> Visual;
struct VisualBase : public OVR::RefCountBase<VisualBase>
{
public :
	typedef VisualBase ValueType;

	virtual ~VisualBase() = 0;
	virtual void Render(OVR::Render::RenderDevice *pRender) = 0;

	Property<bool>			Visibility;
	Property<OVR::Matrix4f> RenderTransform;
	Property<float>			Opacity;
};

struct UIElementBase;
typedef OVR::Ptr<UIElementBase> UIElement;
struct UIElementBase : public VisualBase
{
public:
	float Width() const;
	float Height() const;

	Property<OVR::Sizef> DesiredSize;
	Property<bool> IsMeasureValid; //a value indicating whether the current size returned by layout measure is valid.

	void InvalidMeasure();

	virtual OVR::Sizef Measure(OVR::Sizef DesireSize) const;
	virtual OVR::Sizef Arrange(OVR::Sizef GivenSize) const;

	UIElement* Parent;
};

class FrameworkElementBase;
typedef OVR::Ptr<FrameworkElementBase> FrameworkElement;
class FrameworkElementBase : public UIElementBase
{
public:
	FrameworkElementBase();
	~FrameworkElementBase();

	Property<OVR::Vector4f> Margin;

	Property<HorizentalAlignmentEnum>	HorizentalAlignment;
	Property<VerticalAlignmentEnum>		VerticalAlignment;

private:

};

class PanelBase;
typedef Ptr<PanelBase> Panel;
class PanelBase : public FrameworkElementBase
{
public:
	PanelBase();
	~PanelBase();

	Property<Array<UIElement>> Children;
private:

};

class ContentPresenterBase;
typedef OVR::Ptr<ContentPresenterBase> ContentPresenter;
class ContentPresenterBase : public FrameworkElementBase
{
public:
	ContentPresenterBase();
	~ContentPresenterBase();

private:

};


class HUD :
	public OVR::NewOverrideBase
{
public:
	HUD();

	void Initialize(OVR::Render::RenderDevice *pRender, ovrSizei size);

	void Begin();

	void End();

	OVR::Render::RenderDevice* GetRenderDevice() const;

	void Render();

	~HUD();

	std::vector<OVR::Vector3f> Touches;
	std::mutex m_Mutext;

private:
	ovrSizei							  TextureSize;

	OVR::Render::RenderDevice			 *pDevice;
	OVR::Ptr<OVR::Render::Buffer>         pVertexBuffer;
	OVR::Ptr<OVR::Render::Fill>			  pFill;
	OVR::Ptr<OVR::Render::Texture>        pTexture;
	//OVR::Render::RenderTarget			  RenderTarget;
};

