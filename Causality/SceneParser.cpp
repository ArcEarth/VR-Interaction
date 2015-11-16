#include "pch_bcl.h"
#include "Scene.h"
#include "SceneParser.h"
#include <Model.h>
#include <boost\filesystem.hpp>
#include <tinyxml2.h>
//#include <SkyDome.h>
#include "PlayerProxy.h"
#include "CharacterObject.h"
#include "BasicKeyboardMouseControlLogic.h"
#include "LightObject.h"
#include "SkyDome.h"
#include "Settings.h"
#include "TrackedObjectControl.h"
#include "PenModeler.h"
#include "AssetDictionary.h"

using namespace tinyxml2;
using namespace Causality;
using namespace DirectX::Scene;
using namespace std;

void ParseSceneSettings(tinyxml2::XMLElement * nScene);
void ParseLightObjectAttributes(Light *pLight, tinyxml2::XMLElement * node);
void ParseCameraObjectAttributes(SingleViewCamera *pCamera, tinyxml2::XMLElement * node);
void ParseHmdCameraObjectAttributes(HMDCamera *pCamera, tinyxml2::XMLElement * node);
//void ParseShadowCameraObjectAttributes(SoftShadowCamera *pCamera, tinyxml2::XMLElement * node);
void ParseVisualObjectAttributes(VisualObject* pObj, tinyxml2::XMLElement * node);
void ParseChaacterObjectAttributes(CharacterObject *pCreature, tinyxml2::XMLElement * node);
void ParseSceneObjectAttributes(SceneObject *pObj, XMLElement* node);
void ParseSceneAssets(AssetDictionary& assets, XMLElement* node);

sptr<AssetDictionary::material_type> ParseMaterialAsset(AssetDictionary& assets, XMLElement* node);

AssetDictionary::mesh_type* ParseMeshAsset(AssetDictionary & assets, XMLElement * node);

AssetDictionary::texture_type * ParseTextureAsset(AssetDictionary & assets, XMLElement * node);

AssetDictionary::audio_clip_type * ParseAudioAsset(AssetDictionary & assets, XMLElement * node);

AssetDictionary::armature_type * ParseArmatureAsset(AssetDictionary & assets, XMLElement * node);

AssetDictionary::animation_clip_type* ParseAnimationClipAsset(AssetDictionary & assets, XMLElement * node);

AssetDictionary::behavier_type * ParseBehavierAsset(AssetDictionary & assets, XMLElement * node);

BehavierSpace & LoadBehavierFbx(const char * attr, AssetDictionary & assets, tinyxml2::XMLElement * node);

std::unique_ptr<SceneObject> ParseSceneObject(Scene& scene, XMLElement* node, SceneObject* parent);

#pragma region GetAttribute Helper
template <typename T>
void GetAttribute(_In_ XMLElement* node, _In_ const char* attr, _Inout_ T& value)
{
	static_assert(false, "unsupported type T");
}

template <>
void GetAttribute<Vector3>(_In_ XMLElement* node, _In_  const char* attr, _Inout_ Vector3& value)
{
	auto attrval = node->Attribute(attr);
	if (attrval != nullptr)
	{
		string str(attrval);
		if (str.find_first_of(',') != string::npos)
		{
			stringstream ss(str);
			char ch;
			ss >> value.x >> ch >> value.y >> ch >> value.z;
		}
		else
		{
			value.x = value.y = value.z = (float)atof(attrval); // replicate
		}
	}
}

template <>
void GetAttribute<DirectX::Vector4>(_In_ XMLElement* node, _In_  const char* attr, _Inout_ DirectX::Vector4& value)
{
	auto attrval = node->Attribute(attr);
	if (attrval != nullptr)
	{
		stringstream ss(attrval);
		char ch;
		ss >> value.x >> ch >> value.y >> ch >> value.z >> ch >> value.w;
	}
}


template <>
// Value format : "#AARRGGBB"
void GetAttribute<DirectX::Color>(_In_ XMLElement* node, _In_  const char* attr, _Inout_ DirectX::Color& value)
{
	auto attrval = node->Attribute(attr);
	if (attrval != nullptr && attrval[0] == '#')
	{
		char* end;
		auto val = strtoul(attrval + 1, &end, 16);
		value.w = ((float)((val & 0xff000000U) >> 24)) / (float)0xff;
		value.x = ((float)((val & 0x00ff0000U) >> 16)) / (float)0xff;
		value.y = ((float)((val & 0x0000ff00U) >> 8)) / (float)0xff;
		value.z = ((float)((val & 0x000000ffU) >> 0)) / (float)0xff;
	}
}

template <>
void GetAttribute<bool>(_In_ XMLElement* node, _In_  const char* attr, _Inout_ bool& value)
{
	node->QueryBoolAttribute(attr, &value);
}

template <>
void GetAttribute<float>(_In_ XMLElement* node, _In_  const char* attr, _Inout_ float& value)
{
	node->QueryFloatAttribute(attr, &value);
}

template <>
void GetAttribute<int>(_In_ XMLElement* node, _In_  const char* attr, _Inout_ int& value)
{
	node->QueryIntAttribute(attr, &value);
}

template <>
void GetAttribute<unsigned>(_In_ XMLElement* node, _In_  const char* attr, _Inout_ unsigned& value)
{
	node->QueryUnsignedAttribute(attr, &value);
}

template <>
void GetAttribute<size_t>(_In_ XMLElement* node, _In_  const char* attr, _Inout_ size_t& value)
{
	unsigned ui;
	node->QueryUnsignedAttribute(attr, &ui);
	value = ui;
}

template <>
void GetAttribute<string>(_In_ XMLElement* node, _In_  const char* attr, _Inout_ string& value)
{
	auto attrval = node->Attribute(attr);
	if (attrval != nullptr)
	{
		value = attrval;
	}
}

#pragma endregion

std::unique_ptr<Scene> Scene::LoadSceneFromXML(const string& xml_file)
{
	uptr<Scene> pScene = make_unique<Scene>();
	pScene->LoadFromXML(xml_file);
	return pScene;
}

void Scene::LoadFromXML(const string & xml_file)
{
	using namespace DirectX;
	tinyxml2::XMLDocument sceneDoc;
	auto error = sceneDoc.LoadFile(xml_file.c_str());

	assert(error == XMLError::XML_SUCCESS);

	auto nScene = sceneDoc.FirstChildElement("scene");
	auto nAssets = nScene->FirstChildElement("scene.assets");

	ParseSceneSettings(nScene);
	ParseSceneAssets(this->Assets(), nAssets);

	auto nHud = nScene->FirstChildElement("scene.hud");

	auto nContent = nScene->FirstChildElement("scene.content");
	nContent = nContent->FirstChildElement();
	ParseSceneObject(*this, nContent, nullptr);

	//UpdateRenderViewCache();

	is_loaded = true;
}


void ParseNameText(tinyxml2::XMLElement * setting, const char* name, float& val, float defval)
{
	val = defval;
	auto node = setting->FirstChildElement(name);
	if (node == nullptr) return;
	node->QueryFloatText(&val);
}

void ParseNameText(tinyxml2::XMLElement * setting, const char* name, int& val, int defval)
{
	val = defval;
	auto node = setting->FirstChildElement(name);
	if (node == nullptr) return;
	node->QueryIntText(&val);
}

void ParseNameText(tinyxml2::XMLElement * setting, const char* name, bool& val, bool defval)
{
	val = defval;
	int intval = val;
	auto node = setting->FirstChildElement(name);
	if (node == nullptr) return;
	auto error = node->QueryBoolText(&val);
	if (error != tinyxml2::XMLError::XML_SUCCESS)
	{
		error = setting->FirstChildElement(name)->QueryIntText(&intval);
		val = (intval != 0);
	}
}

void ParseSceneSettings(tinyxml2::XMLElement * nScene)
{
	auto nSettings = nScene->FirstChildElement("scene.settings");

	unsigned clipframe = 90;
	auto child = nSettings->FirstChildElement("ClipRasterizeFrame");
	if (child)
		child->QueryUnsignedText(&clipframe);
	CLIP_FRAME_COUNT = clipframe;

#define SETTING_REGISTERATION(type, name, val) ParseNameText(nSettings,#name,g_##name,val);
#include "SettingsRegisteration.h"
#undef SETTING_REGISTERATION
}

void ParseSceneAssets(AssetDictionary& assets, XMLElement* node)
{
	assets.ParseArchive(node);
}

sptr<AssetDictionary::material_type> ParseMaterialAsset(AssetDictionary & assets, XMLElement * node)
{
	return assets.ParseMaterial(node);
}

AssetDictionary::mesh_type* ParseMeshAsset(AssetDictionary& assets, XMLElement* node)
{
	assets.ParseMesh(node);
}

AssetDictionary::texture_type* ParseTextureAsset(AssetDictionary& assets, XMLElement* node)
{
	assets.ParseTexture(node);
}

AssetDictionary::audio_clip_type* ParseAudioAsset(AssetDictionary& assets, XMLElement* node)
{
	return assets.GetAudio("default");
}

AssetDictionary::armature_type* ParseArmatureAsset(AssetDictionary& assets, XMLElement* node)
{
	return assets.ParseArmature(node);
}

AssetDictionary::animation_clip_type* ParseAnimationClipAsset(AssetDictionary& assets, XMLElement* node)
{
	return assets.ParseAnimation(node);
}

AssetDictionary::behavier_type* ParseBehavierAsset(AssetDictionary& assets, XMLElement* node)
{
	return assets.ParseBehavier(node);
}

void ParseSceneObjectAttributes(SceneObject *pObj, XMLElement* node)
{
	GetAttribute(node, "name", pObj->Name);
	GetAttribute(node, "tag", pObj->Tag);

	Vector3 scale(1.0f);
	Vector3 pos;
	Vector3 eular;

	GetAttribute(node, "position", pos);
	GetAttribute(node, "scale", scale);
	GetAttribute(node, "orientation", eular);

	pObj->SetPosition(pos);
	pObj->SetScale(scale);
	pObj->SetOrientation(Quaternion::CreateFromYawPitchRoll(eular.y, eular.x, eular.z));
}

void ParseSkydomeObjectAttributes(SkyDome* pSkyDome, XMLElement* node)
{
	ParseSceneObjectAttributes(pSkyDome, node);
	auto bg = node->Attribute("background");
	auto& assets = pSkyDome->Scene->Assets();
	auto pEffect = assets.GetEffect("default_environment");

	pSkyDome->CreateDeviceResource(assets.GetRenderDevice(), dynamic_cast<DirectX::EnvironmentMapEffect*>(pEffect));


	if (bg[0] == '{')
	{
		string key(bg + 1, strlen(bg) - 2);
		pSkyDome->SetTexture(*assets.GetTexture(key));
	}
	else
	{
		pSkyDome->SetTexture(*assets.LoadTexture(pSkyDome->Name + "_background", bg));
	}
}

template <typename T>
T* CreateSceneObject(void* ptr)
{
	T* pObj = new T();
	static_cast<SceneObject*>(pObj)->Scene = (Scene*)ptr;
	return pObj;
}

std::unique_ptr<SceneObject> ParseSceneObject(Scene& scene, XMLElement* node, SceneObject* parent)
{
	using namespace DirectX;

	auto& assets = scene.Assets();
	IRenderDevice *device = scene.GetRenderDevice();
	uptr<SceneObject> pObj;

	// parent property
	auto tag = node->Name();
	if (*std::find(tag, tag + strlen(tag), '.'))
		return nullptr;

	auto itr = SceneObject::g_Creators.find(tag);
	if (itr != SceneObject::g_Creators.end())
	{
		auto ptr = itr->second(&scene, node);
		pObj.reset(ptr);
	}
	else
	{
		auto ptr = SceneObject::g_Creators["scene_object"](&scene, node);
		pObj.reset(ptr);
	}

	//if (!strcmp(node->Name(), "object"))
	//{
	//	auto pEntity = CreateSceneObject<VisualObject>(&scene);
	//	ParseVisualObjectAttributes(pEntity, node);
	//	pObj.reset(pEntity);
	//}
	//else if (!strcmp(node->Name(), "camera"))
	//{
	//	auto pCamera = CreateSceneObject<Camera>(&scene);
	//	ParseCameraObjectAttributes(pCamera, node);
	//	pObj.reset(pCamera);
	//}
	//else if (!strcmp(node->Name(), "shadow_camera"))
	//{
	//	auto pCamera = CreateSceneObject<PercentCloserShadowCamera>(&scene);
	//	pCamera->Scene = &scene;
	//	ParseCameraObjectAttributes(pCamera, node);
	//	pObj.reset(pCamera);
	//}
	//else if (!strcmp(node->Name(), "hmd_camera"))
	//{
	//	auto pCamera = CreateSceneObject<HMDCamera>(&scene);
	//	pCamera->Scene = &scene;
	//	ParseHmdCameraObjectAttributes(pCamera, node);
	//	pObj.reset(pCamera);
	//}
	//else if (!strcmp(node->Name(), "light"))
	//{
	//	auto pControl = CreateSceneObject<Light>(&scene);
	//	ParseLightObjectAttributes(pControl, node);
	//	pObj.reset(pControl);
	//}
	//else if (!strcmp(node->Name(), "skydome"))
	//{
	//	auto pControl = CreateSceneObject<Causality::SkyDome>(&scene);
	//	pObj.reset(pControl);
	//	ParseSkydomeObjectAttributes(pControl, node);
	//}
	//else if (!strcmp(node->Name(), "creature"))
	//{
	//	auto pCreature = CreateSceneObject<CharacterObject>(&scene);
	//	ParseChaacterObjectAttributes(pCreature, node);

	//	pObj.reset(pCreature);
	//}
	//else if (!strcmp(node->Name(), "first_person_keyboard_mouse_control"))
	//{
	//	auto pControl = CreateSceneObject<KeyboardMouseFirstPersonControl>(&scene);
	//	pControl->SetTarget(parent);

	//	pObj.reset(pControl);
	//	ParseSceneObjectAttributes(pObj.get(), node);
	//}
	//else if (!strcmp(node->Name(), "coordinate_axis"))
	//{
	//	auto pControl = CreateSceneObject<CoordinateAxis>(&scene);
	//	pObj.reset(pControl);
	//	ParseSceneObjectAttributes(pObj.get(), node);
	//}
	//else if (!strcmp(node->Name(), "player_controller"))
	//{
	//	auto pControl = CreateSceneObject<PlayerProxy>(&scene);
	//	pObj.reset(pControl);
	//	//pObj.reset();
	//	ParseSceneObjectAttributes(pObj.get(), node);
	//}
	//else if (!strcmp(node->Name(), "kinect_visualizer"))
	//{
	//	auto pControl = CreateSceneObject<KinectVisualizer>(&scene);
	//	pObj.reset(pControl);
	//	ParseSceneObjectAttributes(pObj.get(), node);
	//}
	//else if (!strcmp(node->Name(), "tracked_object"))
	//{
	//	auto pControl = CreateSceneObject<TrackedObjectControl>(&scene);
	//	pObj.reset(pControl);
	//	ParseSceneObjectAttributes(pObj.get(), node);
	//}
	//else if (!strcmp(node->Name(), "pen_modeler"))
	//{
	//	auto pControl = CreateSceneObject<PenModeler>(&scene);
	//	pObj.reset(pControl);
	//	ParseSceneObjectAttributes(pObj.get(), node);
	//}
	//else
	//{
	//	pObj.reset(CreateSceneObject<SceneObject>(&scene));
	//	ParseSceneObjectAttributes(pObj.get(), node);
	//}

	if (pObj)
		pObj->Scene = &scene;

	if (parent == nullptr) // this is scene root
	{
		scene.SetContent(pObj.get());
	}
	else
	{
		parent->AddChild(pObj.get());
		std::lock_guard<mutex> guard(scene.ContentMutex());
		scene.SignalCameraCache();
	}

	node = node->FirstChildElement();
	while (node)
	{
		auto pChild = ParseSceneObject(scene, node, pObj.get()).release();
		//{
		//	std::lock_guard<mutex> guard(scene.ContentMutex());
		//	pObj->AddChild(pChild);
		//	//if (pChild->Is<Camera>())
		//	scene.SignalCameraCache();
		//}
		node = node->NextSiblingElement();
	}
	pObj.release();
	return pObj;
}

void ParseChaacterObjectAttributes(CharacterObject* pCreature, tinyxml2::XMLElement * node)
{
	ParseVisualObjectAttributes(pCreature, node);

	auto& assets = pCreature->Scene->Assets();
	auto path = node->Attribute("behavier");
	if (path != nullptr && strlen(path) != 0)
	{
		if (path[0] == '{') // asset reference
		{
			const std::string key(path + 1, path + strlen(path) - 1);
			pCreature->SetBehavier(*assets.GetBehavier(key));
			auto& behavier = pCreature->Behavier();
		}
	}
	else
	{
		auto inlineBehave = node->FirstChildElement("creature.behavier");
		if (inlineBehave)
		{
			inlineBehave = inlineBehave->FirstChildElement();
			auto behavier = ParseBehavierAsset(assets, inlineBehave);
			pCreature->SetBehavier(*behavier);
		}
	}

	auto action = node->Attribute("action");
	if (action)
		pCreature->StartAction(action);
}

void ParseCameraObjectAttributes(SingleViewCamera *pCamera, tinyxml2::XMLElement * node)
{
	using namespace DirectX;
	ParseSceneObjectAttributes(pCamera, node);
	auto rtnode = node->FirstChildElement("camera.render_target");
	bool enable_stereo;
	node->QueryBoolAttribute("enable_stereo", &enable_stereo);
	float fov = 70, aspect = 1, hfov, wfov;
	float _near = 0.1f, _far = 20.0f;
	bool is_primary = false;
	Vector3 focus = (XMVECTOR)pCamera->GetPosition() + XMVector3Rotate(Camera::Foward, pCamera->GetOrientation());
	Vector3 up = g_XMIdentityR1.v;
	Color color = Colors::White.v;
	bool perspective = true;
	GetAttribute(node, "background", color);
	GetAttribute(node, "fov", fov);
	GetAttribute(node, "near", _near);
	GetAttribute(node, "far", _far);
	GetAttribute(node, "focus", focus);
	GetAttribute(node, "up", up);
	GetAttribute(node, "aspect", aspect);
	GetAttribute(node, "primary", is_primary);
	GetAttribute(node, "perspective", perspective);

	auto pRenderControl = pCamera->GetViewRenderer(0, pCamera->ViewRendererCount() - 1);
	auto pEffectRender = dynamic_cast<EffectRenderControl*>(pRenderControl);
	//pEffectRender->SetBackground(color);

	pCamera->FocusAt(focus, up);

	if (perspective)
	{
		pCamera->SetPerspective(XMConvertToRadians(fov), aspect, _near, _far);
	}
	else
	{
		GetAttribute(node, "hfov", hfov);
		GetAttribute(node, "wfov", wfov);
		pCamera->SetOrthographic(wfov, hfov, _near, _far);
	}

	auto pScene = pCamera->Scene;
	if (is_primary)
	{
		pScene->SetAsPrimaryCamera(pCamera);
		auto& canvas = pScene->Canvas();
		pCamera->SetAspectRatio((float)canvas.ColorBuffer().Width() / (float)canvas.ColorBuffer().Height());
		// last renderer of the view
		for (size_t i = 0; i < pCamera->ViewRendererCount(); i++)
		{
			auto pRenderControl = pCamera->GetViewRenderer(0, i);
			auto pEffectRender = dynamic_cast<EffectRenderControl*>(pRenderControl);
			if (pEffectRender->GetRenderTarget().ColorBuffer() == nullptr)
			{
				pEffectRender->SetRenderTarget(canvas);
			}
			//if (pEffectRender->GetOutput() == nullptr && pEffectRender->GetPostEffect() != nullptr)
			//{
			//	pEffectRender->SetPostEffectOutput(canvas.ColorBuffer());
			//}
		}
	}

	if (rtnode)
	{
		int width, height;
		rtnode = rtnode->FirstChildElement("render_target");
		GetAttribute(rtnode, "width", width);
		GetAttribute(rtnode, "height", height);
		auto device = pScene->GetRenderDevice();
		pEffectRender->SetRenderTarget(RenderTarget(device, width, height));
	}
}

void ParseHmdCameraObjectAttributes(HMDCamera *pCamera, tinyxml2::XMLElement * node)
{
	using namespace DirectX;
	ParseSceneObjectAttributes(pCamera, node);
	auto rtnode = node->FirstChildElement("camera.render_target");
	bool enable_stereo;
	node->QueryBoolAttribute("enable_stereo", &enable_stereo);
	float fov = 70, aspect = 1, hfov, wfov;
	float _near = 0.1f, _far = 20.0f;
	bool is_primary = false;
	Vector3 focus = (XMVECTOR)pCamera->GetPosition() + XMVector3Rotate(Camera::Foward, pCamera->GetOrientation());
	Vector3 up = g_XMIdentityR1.v;
	Color color = Colors::White.v;
	bool perspective = true;
	float ipd = 0.065;
	GetAttribute(node, "background", color);
	GetAttribute(node, "fov", fov);
	GetAttribute(node, "near", _near);
	GetAttribute(node, "far", _far);
	GetAttribute(node, "focus", focus);
	GetAttribute(node, "up", up);
	GetAttribute(node, "aspect", aspect);
	GetAttribute(node, "primary", is_primary);
	GetAttribute(node, "perspective", perspective);
	GetAttribute(node, "ipd", ipd);
	pCamera->SetIPD(ipd);

	auto pRenderControl = pCamera->GetViewRenderer(0, pCamera->ViewRendererCount() - 1);
	auto pEffectRender = dynamic_cast<EffectRenderControl*>(pRenderControl);
	//pEffectRender->SetBackground(color);

	for (int i = 0; i < pCamera->ViewCount(); i++)
	{
		auto pView = dynamic_cast<CameraViewControl*>(pCamera->GetView(i));
		pView->FocusAt(focus, up);

		if (perspective)
		{
			pView->SetPerspective(XMConvertToRadians(fov), aspect, _near, _far);
		}
		else
		{
			GetAttribute(node, "hfov", hfov);
			GetAttribute(node, "wfov", wfov);
			pView->SetOrthographic(wfov, hfov, _near, _far);
		}

		auto pScene = pCamera->Scene;
		if (is_primary)
		{
			pScene->SetAsPrimaryCamera(pCamera);
			auto& canvas = pScene->Canvas();
			pView->SetAspectRatio((float)canvas.ColorBuffer().Width() / (float)canvas.ColorBuffer().Height());
			// last renderer of the view
			for (size_t i = 0; i < pCamera->ViewRendererCount(); i++)
			{
				auto pRenderControl = pCamera->GetViewRenderer(0, i);
				auto pEffectRender = dynamic_cast<EffectRenderControl*>(pRenderControl);
				if (pEffectRender->GetRenderTarget().ColorBuffer() == nullptr)
				{
					pEffectRender->SetRenderTarget(canvas);
				}
			}
		}

		if (rtnode)
		{
			int width, height;
			rtnode = rtnode->FirstChildElement("render_target");
			GetAttribute(rtnode, "width", width);
			GetAttribute(rtnode, "height", height);
			auto device = pScene->GetRenderDevice();
			pEffectRender->SetRenderTarget(RenderTarget(device, width, height));
		}
	}
}


void ParseLightObjectAttributes(Light* pLight, tinyxml2::XMLElement * node)
{
	using namespace DirectX;
	ParseCameraObjectAttributes(pLight, node);
	unsigned resolution = 1024;
	bool enableShadow;
	Color color = Colors::White.v;
	GetAttribute(node, "color", color);
	GetAttribute(node, "drops_shadow", enableShadow);
	GetAttribute(node, "resolution", resolution);

	pLight->SetColor(color);
	if (enableShadow)
	{
		auto device = pLight->Scene->GetRenderDevice();
		pLight->EnableDropShadow(device, resolution);
	}
}


void ParseVisualObjectAttributes(VisualObject* pObj, tinyxml2::XMLElement * node)
{
	ParseSceneObjectAttributes(pObj, node);

	float mass = 1.0f;
	GetAttribute(node, "mass", mass);

	auto& assets = pObj->Scene->Assets();
	auto path = node->Attribute("mesh");
	if (path != nullptr && strlen(path) != 0)
	{
		if (path[0] == '{') // asset reference
		{
			const std::string key(path + 1, path + strlen(path) - 1);
			pObj->SetRenderModel(assets.GetMesh(key));
		}
	}
	else
	{
		auto nMesh = node->FirstChildElement("object.mesh");
		if (nMesh)
		{
			nMesh = nMesh->FirstChildElement();
			auto model = ParseMeshAsset(assets, nMesh);
			pObj->SetRenderModel(model);
		}
	}
}