#include "DirectXMathExtend.h"
#include <d3d11.h>
#include <vector>
#include <wrl\client.h>
#include <memory>
#include "Textures.h"
#include <boost\any.hpp>

namespace DirectX
{
	namespace Scene
	{
		// Abstraction for the requirement about Pixel shader process
		class IMaterial abstract
		{
		public:
			virtual IEffect* GetRequestedEffect() const = 0;

			virtual ~IMaterial() {}
			//virtual Color	GetColor(const char* key) const = 0;
			//virtual float	GetFloat(const char* key) const = 0;
			//virtual int		GetInt(const char* key) const = 0;
			//virtual int		GetString(const char* key) const = 0;
			//virtual ID3D11ShaderResourceView*	GetTexture(const char* key) const = 0;
			//virtual Color	SetColor(const char* key) const = 0;
			//virtual void SetFloat(const char* key) const = 0;
			//virtual void SetInt(const char* key) const = 0;
			//virtual void SetString(const char* key) const = 0;
			//virtual void SetTexture(const char* key) const = 0;
		};

		//0. Color on and Ambient off
		//1. Color on and Ambient on
		//2. Highlight on
		//3. Reflection on and Ray trace on
		//4. Transparency: Glass on, Reflection : Ray trace on
		//5. Reflection : Fresnel on and Ray trace on
		//6. Transparency : Refraction on, Reflection : Fresnel off and Ray trace on
		//7. Transparency : Refraction on, Reflection : Fresnel on and Ray trace on
		//8. Reflection on and Ray trace off
		//9. Transparency : Glass on, Reflection : Ray trace off
		//10. Casts shadows onto invisible surfaces
		enum ObjMateriaIlluminitionModel
		{
			ColorOnAmbientOff = 0,
			ColorOnAmbientOn = 1,
			HighlightOn = 2,
			ReflectionOnRayTraceOn = 3,
			TransparencyOn = 4,
		};

		class PropertyMap : private std::map<std::string, boost::any>
		{
		public:
			typedef std::map<std::string, boost::any> base_type;
			template <typename T>
			T	Get(const std::string& key) const
			{
				auto itr = find(key);
				if (itr != end())
					return boost::any_cast<T>(itr->second);
				else 
					return T();
			}

			template <typename T>
			void Set(const std::string& key, const T& value)
			{
				base_type::operator[](key) = value;
			}

			using base_type::operator[];

			bool HasProperty(const std::string& key) const
			{
				return find(key) != end();
			}

			using base_type::begin;
			using base_type::end;
			using base_type::size;

			size_t Size() const { return size(); }

			const std::map<std::string, boost::any>& Properties() const 
			{ return *this; }
		};

		class IPhongMaterial abstract
		{
			// Phong model
			virtual Color GetAmbientColor() const = 0;
			//virtual void SetAmbientColor(const Color& color) = 0;
			virtual Color GetDiffuseColor() const = 0;
			//virtual void SetDiffuseColor(const Color& color) = 0;
			virtual Color GetSpecularColor() const = 0;
			//virtual void SetSpecularColor(const Color& color) = 0;
			virtual float GetAlpha() const = 0;
			//virtual void SetAlpha(float alpha) const = 0;

			virtual ID3D11ShaderResourceView *GetDiffuseMap() const = 0;
			virtual ID3D11ShaderResourceView *GetNormalMap() const = 0;
			virtual ID3D11ShaderResourceView *GetDisplaceMap() const = 0;
			virtual ID3D11ShaderResourceView *GetSpecularMap() const = 0;
		};

		class Material : public IMaterial, public PropertyMap, public IPhongMaterial
		{
		public:
			std::string RequstedEffectName;

			Color		GetColor(const std::string& key) const
			{
				return Get<Color>(key);
			}
			float		GetFloat(const std::string& key) const
			{
				return Get<float>(key);
			}
			int			GetInt(const std::string& key) const
			{
				return Get<int>(key);
			}
			Vector4		GetVector4(const std::string& key) const
			{
				return Get<Vector4>(key);
			}
			std::string	GetString(const std::string& key) const
			{
				return Get<std::string>(key);
			}
			const Texture&	GetTexture(const std::string& key) const
			{
				return *Get<const Texture*>(key);
			}

			Color		                GetAmbientColor() const { return GetColor("AmbientColor"); }
			Color		                GetDiffuseColor() const { return GetColor("DiffuseColor"); }
			Color		                GetSpecularColor() const { return GetColor("SpecularColor"); }
			float		                GetSpecularPower() const { return GetFloat("SpecularPower"); }
			float		                GetOpacity() const { return GetFloat("Opacity"); }
			ID3D11ShaderResourceView*	GetDiffuseMap() const override { return GetTexture("DiffuseMap"); }
			ID3D11ShaderResourceView*	GetNormalMap() const override { return GetTexture("NormalMap"); }
			ID3D11ShaderResourceView*	GetDisplaceMap() const override { return GetTexture("DisplaceMap"); }
			ID3D11ShaderResourceView*	GetSpecularMap() const override { return GetTexture("SpecularMap"); }

			void		SetColor(const std::string& key, const Color& value)
			{
				Set(key,value);
			}
			void		SetFloat(const std::string& key, float value)
			{
				Set(key, value);
			}
			void		SetInt(const std::string& key, int value)
			{
				Set(key, value);
			}
			void		SetString(const std::string& key, const std::string& value)
			{
				Set(key, value);
			}
			void		SetTexture(const std::string& key, Texture& texture)
			{
				Set(key, &texture);
			}
			void		SetVector4(const std::string& key, const Vector4 value)
			{
				Set(key, value);
			}
		};

		struct PhongMaterial : public Material
		{
		public:
			using SRVComPtr = Microsoft::WRL::ComPtr<ID3D11ShaderResourceView>;

			PhongMaterial();
			static std::vector<std::shared_ptr<PhongMaterial>> CreateFromMtlFile(ID3D11Device* pDevice, const std::wstring &file, const std::wstring &lookupDirectory);

			IEffect*	Effect;
			std::string Name;
			Color		AmbientColor;
			Color		DiffuseColor;
			Color		SpecularColor;
			Color		EmmsiveColor;
			float		SpecularPower;
			float		Alpha;

			SRVComPtr	DiffuseMap;
			SRVComPtr	NormalMap;
			SRVComPtr	DisplaceMap;
			SRVComPtr	SpecularMap;

			// Inherited via IMaterial
			virtual IEffect* GetRequestedEffect() const override { return Effect; }
			virtual Color GetAmbientColor() const override;
			virtual Color GetDiffuseColor() const override;
			virtual Color GetSpecularColor() const override;
			virtual float GetAlpha() const override;
			virtual ID3D11ShaderResourceView * GetDiffuseMap() const override;
			virtual ID3D11ShaderResourceView * GetNormalMap() const override;
			virtual ID3D11ShaderResourceView * GetSpecularMap() const override;
			virtual ID3D11ShaderResourceView * GetDisplaceMap() const override;
		};
	}
}