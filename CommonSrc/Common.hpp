#pragma once
#pragma warning(push)
#pragma warning(disable: 4512)

#include <vector>
#include <memory>
#include <functional>
#include <boost\signals2.hpp>
#include <wrl\client.h>

namespace Platform
{
	template <class T>
	using sptr = std::shared_ptr<T>;

	template <class T>
	using uptr = std::unique_ptr<T>;

	template <class T>
	using cptr = Microsoft::WRL::ComPtr<T>;

	// TEMPLATE FUNCTION refnew , wrapper for std::make_shard
	template<class _Ty,
	class... _Types> inline
		sptr<_Ty> newref(_Types&&... _Args)
	{	// make a shared_ptr
		return std::make_shared<_Ty>(_STD forward<_Types>(_Args)...);
	}

	// TEMPLATE FUNCTION uninew, wrapper for std::make_unique
	template<class _Ty,
	class... _Types> inline
		uptr<_Ty> newuni(_Types&&... _Args)
	{	// make a shared_ptr
		return std::make_unique<_Ty>(_STD forward<_Types>(_Args)...);
	}

	// TEMPLATE FUNCTION uninew, wrapper for std::make_unique
	template<class _Ty,
	class... _Types> inline
		cptr<_Ty> newcom(_Types&&... _Args)
	{	// make a shared_ptr
		return Microsoft::WRL::Make<_Ty>(_STD forward<_Types>(_Args)...);
	}

	template <class TArg>
	using Event = boost::signals2::signal<void(TArg)>;

	template <class TArg,class TCallback>
	inline void operator+=(Event<TArg>& signal, TCallback callback)
	{
		signal.connect(callback);
	}

	template <class TArg, class TCallback>
	inline void operator-=(Event<TArg>& signal, TCallback callback)
	{
		signal.disconnect(callback);
	}

	class scope_exit
	{
	public:
		scope_exit(std::function<void()> action)
			: _action(std::move(action))
		{
		}
		~scope_exit()
		{
			_action();
		}
	private:
		std::function<void()> _action;
	};

	template <class T> void SafeDelete(_Inout_ T*&pT)
	{
		if (pT)
			delete pT;
		pT = nullptr;
	}


}
#pragma warning(pop)