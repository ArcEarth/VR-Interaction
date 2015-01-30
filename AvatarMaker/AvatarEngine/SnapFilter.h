#pragma once

#include <list>
#include <map>
#include <deque>
#include <type_traits>
#include <cmath>

namespace Filters
{
	template <class T>
	inline T sgn (const T& value)
	{
		return (value>T(0)) ? T(1) : T(-1);
	}

	template <class T>
	class SnapFilter
	{
	public:
		typedef typename std::remove_reference<T>::type ValueType;
		typedef ValueType& ReferenceType;
		typedef const ValueType& ConstReferenceType;
	public:

		SnapFilter(size_t rank = 1)
			: Rank(rank) , CurrentKey(Keys.end())
		{
		}

		struct ValuePairType
		{
			ValueType raw;
			ValueType filtered;
			ValuePairType(ConstReferenceType _raw,ConstReferenceType _filtered)
				: raw(_raw) , filtered(_filtered)
			{}
		};

		ValueType Apply(ConstReferenceType rawValue);
		void ResetDynamic();

		bool InsertKey(ConstReferenceType Key , ConstReferenceType SnapRadius);
		size_t RemoveKey(ConstReferenceType Key);
		size_t RemoveKey(ConstReferenceType Key , ConstReferenceType AffectRadius);
		void ClearKey();

	protected:
		typedef std::map<ValueType,ValueType>			KeyMapType;
		KeyMapType										Keys;
		std::deque<ValuePairType>						History;
		size_t											Rank;
		typename KeyMapType::const_iterator					CurrentKey;
	};

	template <class T>
	bool SnapFilter<T>::InsertKey(ConstReferenceType Key , ConstReferenceType SnapRadius)
	{
		using namespace std;
		for (auto& key : Keys)
		{
			if (abs(key.first - Key) < max(key.second,SnapRadius))
				return false;
		}
		Keys.insert(make_pair(Key,SnapRadius));
		return true;
	}

	template <class T>
	size_t SnapFilter<T>::RemoveKey(ConstReferenceType Key)
	{
		if (CurrentKey != Keys.end() && CurrentKey->first == Key)
			CurrentKey = Keys.end();
		return Keys.erase(Key) ;
	}

	template <class T>
	void SnapFilter<T>::ClearKey()
	{
		Keys.clear();
		CurrentKey = Keys.end();
	}

	template <class T>
	void SnapFilter<T>::ResetDynamic()
	{
		History.clear();
		CurrentKey = Keys.end();
	}

	template <class T>
	typename SnapFilter<T>::ValueType SnapFilter<T>::Apply(ConstReferenceType rawValue)
	{
		if (History.size() < Rank)
		{
			History.emplace_front(rawValue,rawValue);
			return rawValue;
		}

		const auto& lastValue = History.back().filtered;
		auto delta = rawValue - History.back().raw;
		auto value = lastValue + delta;
		if (CurrentKey == Keys.end())
		{
			for (auto key = Keys.begin(); key != Keys.end(); key++)
			{
				if (((lastValue < key->first) != (value < key->first)) &&
					(std::abs(value - key->first) < key->second))
				{
						CurrentKey = key;
						value = CurrentKey->first;
						History.emplace_front(rawValue,value);
						if (History.size() > Rank) History.pop_back();
						return value;
				}
			}
			History.emplace_front(rawValue,value);
			if (History.size() > Rank) History.pop_back();
			return value;
		} else
		{
			auto& key = CurrentKey;
			if ((std::abs(value - key->first) < key->second))
			{
				History.emplace_front(rawValue,value);
				if (History.size() > Rank) History.pop_back();
				return CurrentKey->first;
			} else
			{
				value += sgn(key->first - value)*key->second;
				History.emplace_front(rawValue,value);
				if (History.size() > Rank) History.pop_back();
				CurrentKey = Keys.end();
				return value;
			}
		}
	}


}
