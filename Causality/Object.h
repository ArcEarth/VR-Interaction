#pragma once
#include "Common\BCL.h"

namespace Causality
{
	class Object
	{
	public:
		id_t						ID;
		string						Name;

		template <typename T>
		bool Is() const
		{
			return dynamic_cast<const T*>(this) != nullptr;
		}

		template <typename T>
		const T& As() const
		{
			return dynamic_cast<const T&>(*this);
		}

		template <typename T>
		T& As()
		{
			return dynamic_cast<T&>(*this);
		}

		virtual ~Object();
	};
}