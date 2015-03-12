#include "DirectXMathExtend.h"
#include <d3d11.h>
#include <vector>
#include <wrl\client.h>
#include <memory>
#include "Textures.h"

namespace DirectX
{
	namespace Scene
	{
		// Abstraction for the requirement about Pixel shader process
		class IMaterial abstract
		{
		public:
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

		struct PhongMaterial : public IPhongMaterial, public IMaterial
		{
		public:
			PhongMaterial();
			static std::vector<std::shared_ptr<PhongMaterial>> CreateFromMtlFile(ID3D11Device* pDevice, const std::wstring &file, const std::wstring &lookupDirectory);

			std::string Name;
			Color AmbientColor;
			Color DiffuseColor;
			Color SpecularColor;
			Color EmmsiveColor;
			float SpecularPower;
			float Alpha;

			Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> DiffuseMap;
			Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> NormalMap;
			Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> DisplaceMap;
			Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> SpecularMap;

			// Inherited via IMaterial
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