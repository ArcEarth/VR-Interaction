#pragma once
#include "BCL.h"

namespace Causality
{
	// object is managed resource that maybe "delayed destroy"
	class Object
	{
	public:
		id_t						ID;

		template <typename T>
		bool Is() const
		{
			return dynamic_cast<const T*>(this) != nullptr;
		}

		template <typename T>
		const T* As() const
		{
			return dynamic_cast<const T&>(this);
		}

		template <typename T>
		T* As()
		{
			return dynamic_cast<T*>(this);
		}

		virtual ~Object() = default;
	};

	class ObjectManager
	{
	public:
		static ObjectManager GlobalObjectManager;

	private:
		std::unordered_map<id_t, Object*> objects_map;
	};
}