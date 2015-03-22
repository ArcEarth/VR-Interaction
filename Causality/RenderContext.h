#pragma once
#include <d3d11_2.h>
#include <wrl\client.h>

namespace Causality
{
	class RenderDevice : public Microsoft::WRL::ComPtr<ID3D11Device>
	{
	public:
		typedef Microsoft::WRL::ComPtr <ID3D11Device> base_type;
		using base_type::operator->;
		using base_type::operator&;
		using base_type::operator=;
		using base_type::operator Microsoft::WRL::Details::BoolType;

		RenderDevice() = default;
		RenderDevice(ID3D11Device* pDevice)
			: base_type(pDevice)
		{
		}

		operator ID3D11Device*()
		{
			return Get();
		}
	};

	class RenderContext : public Microsoft::WRL::ComPtr<ID3D11DeviceContext>
	{
	public:
		typedef Microsoft::WRL::ComPtr <ID3D11DeviceContext> base_type;
		using base_type::operator->;
		using base_type::operator&;
		using base_type::operator=;
		using base_type::operator Microsoft::WRL::Details::BoolType;

		RenderContext() = default;
		RenderContext(ID3D11DeviceContext* pContext)
			: base_type(pContext)
		{}

		operator ID3D11DeviceContext*()
		{
			return Get();
		}
	};

}