#pragma	once
#ifndef SAFE_DELETE_H
#define SAFE_DELETE_H

template< class T > void SafeDelete( T*& pVal )
{
	if (pVal)
	{
		delete pVal;
	}
	pVal = nullptr;
}

template< class T > void SafeDeleteArray( T*& pVal )
{
	if (pVal)
	{
		delete[] pVal;
	}
	pVal = nullptr;
}

#endif // #endif SAFE_DELETE_H