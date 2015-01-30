#pragma once

#include <iostream>

class ISerializable
{
public:
	struct Blob
	{
		size_t size;
		unsigned char* pData;

		Blob()
			: size(0) , pData(nullptr)
		{}

		void Release()
		{
			if (pData) delete pData;
			size = 0;
		}

	};

	virtual Blob Serialize() const = 0;
	virtual void Deserialize(const Blob& Data) = 0; 
};

inline std::ostream& operator << (std::ostream& lhs, const ISerializable::Blob& rhs)
{
	lhs << rhs.size <<'\0';
	lhs.write(reinterpret_cast<const char*>(rhs.pData),rhs.size);
	return lhs;
};

inline std::istream& operator >> (std::istream& lhs, ISerializable::Blob& rhs)
{
	size_t size;
	char seperator;
	lhs >> size;
	lhs >> seperator; 
	if (rhs.size < size)
	{
		if (rhs.pData) 
			delete rhs.pData;
		rhs.pData = new unsigned char[size];
	}

	rhs.size = size;
	lhs.read(reinterpret_cast<char*>(rhs.pData),size);
	return lhs;
};
