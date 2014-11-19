#pragma once
#include <iterator>
#include <cstdint>
#include <type_traits>

template <class T>
class stride_iterator
{
private:
	T* data;
	// stride in byte
	size_t stride;

protected:
	// these friend declarations are to allow access to the protected
	// "raw" constructor that starts from a raw pointer plus
	// stride+length info
	template <class U> friend class stride_iterator;

	stride_iterator(T* data, size_t stride) :
		data(d), stride(str) {}

public:
	typedef std::random_access_iterator_tag iterator_category;

	stride_iterator(const stride_iterator& other) :
		data(other.data), stride(other.stride) {}

	template <class U>
	explicit stride_iterator(const stride_iterator<U>& other) :
		data(other.data), stride(other.stride) {}

	typedef T            value_type;
	typedef ptrdiff_t    difference_type;
	typedef T*           pointer;
	typedef T&           reference;

	bool has_more() const { return data < stop; }

	// Dereference

	reference operator*() const 
	{ 
#if _ITERATOR_DEBUG_LEVEL == 2
		if (stop <= data)
		{	// report error
			_DEBUG_ERROR("random access iterator subscript out of range");
			_SCL_SECURE_OUT_OF_RANGE;
		}

#elif _ITERATOR_DEBUG_LEVEL == 1
		_SCL_SECURE_VALIDATE_RANGE(data < stop);
#endif /* _ITERATOR_DEBUG_LEVEL */
		return *data;
	}
	reference operator[](int idx)
	{
		auto ptr = reinterpret_cast<T*>(reinterpret_cast<char*>(data) + stride*idx);
		return *ptr;
	}
	const reference operator[](int idx) const
	{
		auto ptr = reinterpret_cast<const T*>(reinterpret_cast<const char*>(data) + stride*idx);
		return *ptr;
	}

	// Comparison

	bool operator==(const stride_iterator& other) const { return data == other.data; }

	bool operator!=(const stride_iterator& other) const { return data != other.data; }

	bool operator<(const stride_iterator& other) const { return data < other.data; }

	difference_type operator-(const stride_iterator& other) const
	{
		return ((char*)data - (char*)other.data) / stride;
	}

	// Increment/Decrement

	stride_iterator& operator++() { data = reinterpret_cast<T*>(reinterpret_cast<char*>(data) + stride); return *this; }
	stride_iterator& operator--() { data = reinterpret_cast<T*>(reinterpret_cast<char*>(data) - stride); return *this; }

	stride_iterator operator++(int) { stride_iterator cpy(*this); data += stride; return cpy; }
	stride_iterator operator--(int) { stride_iterator cpy(*this); data -= stride; return cpy; }

	stride_iterator& operator+=(int x) { data = &(*this)[x]; return *this; }
	stride_iterator& operator-=(int x) { data = &(*this)[-x]; return *this; }

	stride_iterator operator+(int x) const { stride_iterator res(*this); res += x; return res; }
	stride_iterator operator-(int x) const { stride_iterator res(*this); res -= x; return res; }
};

template <class T>
class stride_range
{
public:
	typedef std::remove_reference_t<T>				value_type;
	typedef ptrdiff_t								difference_type;
	typedef value_type*								pointer;
	typedef value_type&								reference;
	typedef stride_iterator<value_type>				iterator_type;
	typedef stride_iterator<std::add_const_t<T>>	const_iterator_type;
protected:
	pointer data;
	// stride in byte
	size_t	stride;
	pointer stop;
public:
	stride_range(T* data, size_t stride, size_t count)
		:data(data),stride(stride),stop(reinterpret_cast<T*>(reinterpret_cast<char*>(data) + stride*count))
	{}

	iterator_type begin()
	{
		return iterator_type(data, stride);
	}

	iterator_type end()
	{
		return iterator_type(stop, stride);
	}

	const_iterator_type begin() const
	{
		return const_iterator_type(data, stride);
	}

	const_iterator_type end() const
	{
		return const_iterator_type(stop, stride);
	}

	const_iterator_type cbegin() const
	{
		return const_iterator_type(data, stride);
	}

	const_iterator_type cend() const
	{
		return const_iterator_type(stop, stride);
	}

	reference operator[](int idx)
	{
		auto ptr = reinterpret_cast<T*>(reinterpret_cast<char*>(data) + stride*idx);
#if _ITERATOR_DEBUG_LEVEL == 2
		if (stop <= ptr)
		{	// report error
			_DEBUG_ERROR("vector subscript out of range");
			_SCL_SECURE_OUT_OF_RANGE;
		}

#elif _ITERATOR_DEBUG_LEVEL == 1
		_SCL_SECURE_VALIDATE_RANGE(ptr < stop);
#endif /* _ITERATOR_DEBUG_LEVEL */
		return *ptr;
	}

	const reference operator[](int idx) const
	{
		auto ptr = reinterpret_cast<const T*>(reinterpret_cast<const char*>(data) + stride*idx);
#if _ITERATOR_DEBUG_LEVEL == 2
		if (stop <= ptr)
		{	// report error
			_DEBUG_ERROR("vector subscript out of range");
			_SCL_SECURE_OUT_OF_RANGE;
		}

#elif _ITERATOR_DEBUG_LEVEL == 1
		_SCL_SECURE_VALIDATE_RANGE(ptr < stop);
#endif /* _ITERATOR_DEBUG_LEVEL */
		return *ptr;
	}

};