#pragma once
#include "BCL.h"
#include <d3d11_2.h>

namespace Causality
{
	class IRenderDevice abstract
	{

	};

	class IRenderContext abstract
	{

	};

	class RenderDevice : public cptr<ID3D11Device>
	{
	public:
		typedef cptr<ID3D11Device> base_type;
		using base_type::operator->;
		using base_type::operator&;
		using base_type::operator=;
		using base_type::operator Microsoft::WRL::Details::BoolType;

		RenderDevice();

		RenderDevice(ID3D11Device* pDevice)
			: base_type(pDevice)
		{
		}

		operator ID3D11Device*()
		{
			return Get();
		}
	};

	class RenderContext : public cptr<ID3D11DeviceContext>
	{
	public:
		typedef cptr<ID3D11DeviceContext> base_type;
		using base_type::operator->;
		using base_type::operator&;
		using base_type::operator=;
		using base_type::operator Microsoft::WRL::Details::BoolType;

		RenderContext() {}
		RenderContext(ID3D11DeviceContext* pContext)
			: base_type(pContext)
		{}

		operator ID3D11DeviceContext*()
		{
			return Get();
		}
	};

}