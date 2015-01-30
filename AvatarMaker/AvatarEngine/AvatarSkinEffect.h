#pragma once

#include <Effects.h>

namespace DirectX
{
	class AvatarSkinnedEffect
		: public DirectX::IEffect ,public DirectX::IEffectMatrices ,public DirectX::IEffectLights
	{
	public:
		explicit AvatarSkinnedEffect(_In_ ID3D11Device* device);
		virtual ~AvatarSkinnedEffect();

			// IEffect methods.
			void Apply(_In_ ID3D11DeviceContext* deviceContext) override;

		void GetVertexShaderBytecode(_Out_ void const** pShaderByteCode, _Out_ size_t* pByteCodeLength) override;

		// Camera settings.
		virtual void XM_CALLCONV SetWorld(FXMMATRIX value) override;
		virtual void XM_CALLCONV SetView(FXMMATRIX value) override;
		virtual void XM_CALLCONV SetProjection(FXMMATRIX value) override;

		//// Material settings.
		//void SetDiffuseColor(FXMVECTOR value);
		//void SetEmissiveColor(FXMVECTOR value);
		//void SetSpecularColor(FXMVECTOR value);
		//void SetSpecularPower(float value);
		//void SetAlpha(float value);

		void EnableIndicatorView(bool value);

		// Light settings.
		void SetPerPixelLighting(bool value) override;
		void XM_CALLCONV SetAmbientLightColor(FXMVECTOR value) override;

		void SetLightEnabled(int whichLight, bool value) override;
		void XM_CALLCONV SetLightDirection(int whichLight, FXMVECTOR value) override;
		void XM_CALLCONV SetLightDiffuseColor(int whichLight, FXMVECTOR value) override;
		void XM_CALLCONV SetLightSpecularColor(int whichLight, FXMVECTOR value) override;

		void EnableDefaultLighting() override;

	private:
		class Impl;
		std::unique_ptr<Impl> m_pImpl;
	};
}
