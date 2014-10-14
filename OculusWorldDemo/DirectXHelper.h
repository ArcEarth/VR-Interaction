#pragma once

#ifndef __d3d11_1_h__
#error You should include d3d11_1.h before include this header
#else //!__d3d11_1_h__
#include <ppl.h>
#include <ppltasks.h>
#include <string>
#include <fstream>
#include <streambuf>
#include <exception>
#include <DirectXMath.h>
#include <sstream>
#include <wrl\client.h>

namespace DirectX
{

	inline std::wstring ErrorDescription(HRESULT hr) 
	{ 
		if(FACILITY_WINDOWS == HRESULT_FACILITY(hr)) 
			hr = HRESULT_CODE(hr); 
		TCHAR* szErrMsg; 

		auto size = FormatMessage( 
			FORMAT_MESSAGE_ALLOCATE_BUFFER|FORMAT_MESSAGE_FROM_SYSTEM, 
			NULL, hr, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), 
			(LPTSTR)&szErrMsg, 0, NULL);

		if (!size){
			std::wstring msg(szErrMsg);
			LocalFree(szErrMsg); 
			return msg;
		} else
		{
			std::wstringstream sstr;
			sstr<<"Undefined Error Code ("<<hr<<").\n";
			return sstr.str();
		}
	}



	inline void ThrowIfFailed(HRESULT hr)
	{
		if (FAILED(hr))
		{
			auto ErrMsg = ErrorDescription(hr);
			std::string	Msg(ErrMsg.begin(), ErrMsg.end());
			throw std::exception(Msg.c_str());
		}
	}

	template <typename T>
	inline void SafeRelease(T* &pData){
		if (pData) {
			pData->Release();
			pData = nullptr;
		}
	}


	inline std::string ReadFileToString(const char *pFileName){
		std::ifstream fin;
		fin.open(pFileName, std::ios::in | std::ios::binary);
		if (!fin.is_open())
			throw new std::exception("File not found exception.");
		std::string str((std::istreambuf_iterator<char>(fin)), std::istreambuf_iterator<char>());
		return str;
	}

	inline std::string ReadFileToString(const std::wstring& fileName){
		std::ifstream fin;
		fin.open(fileName, std::ios::in | std::ios::binary);
		if (!fin.is_open())
			throw new std::exception("File not found exception.");
		std::string str((std::istreambuf_iterator<char>(fin)), std::istreambuf_iterator<char>());
		return str;
	}

	inline Concurrency::task<std::string> ReadDataAsync(const std::wstring& fileName)
	{
		Concurrency::task<std::string> RetriveDateTask([&fileName]()->std::string{
			std::ifstream fin(fileName);
			if (!fin)
			{
				return std::string((std::istreambuf_iterator<char>(fin)), std::istreambuf_iterator<char>());
			} else
			{
				throw std::exception("File not found exception.");
			}
		});
	}

	template <typename ConstantType>
	inline ID3D11Buffer* CreateConstantBuffer(ID3D11Device *pDevice )
	{
		D3D11_BUFFER_DESC desc = { 0 };

		desc.ByteWidth = sizeof(ConstantType);
		desc.Usage = D3D11_USAGE_DYNAMIC;
		desc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
		desc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
		ID3D11Buffer* pBuffer = nullptr;
		ThrowIfFailed(
			pDevice->CreateBuffer(&desc,nullptr,&pBuffer)
		);

		return pBuffer;
	}

	template <typename T>
	void SetBufferData(ID3D11Buffer *pBuffer , _In_ ID3D11DeviceContext* pContext, T const& value)
	{
		D3D11_MAPPED_SUBRESOURCE mappedResource;
			
		ThrowIfFailed(
			pContext->Map(pBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &mappedResource)
		);

		*(T*)mappedResource.pData = value;

		pContext->Unmap(pBuffer, 0);
	}


	template <class VertexType>
	inline ID3D11Buffer* CreateVertexBuffer(ID3D11Device *pDevice ,int Capablity ,const VertexType* pInitialData ,UINT CPUAccessFlag = 0)
	{
		CD3D11_BUFFER_DESC VertexBufferDesc(sizeof(VertexType)*Capablity, D3D11_BIND_VERTEX_BUFFER, D3D11_USAGE_DEFAULT, CPUAccessFlag);
		ID3D11Buffer* pBuffer = nullptr;
		if (pInitialData){
			D3D11_SUBRESOURCE_DATA InitialSubresource;
			InitialSubresource.pSysMem = pInitialData;
			InitialSubresource.SysMemPitch = 0;
			InitialSubresource.SysMemSlicePitch = 0;
			ThrowIfFailed(
				pDevice->CreateBuffer(
					&VertexBufferDesc,
					&InitialSubresource,
					&pBuffer
					)
				);
		} else	{
			ThrowIfFailed(
				pDevice->CreateBuffer(
					&VertexBufferDesc,
					nullptr,
					&pBuffer
					)
				);
		}
		return pBuffer;
	}

	template <class IndexType>
	inline ID3D11Buffer* CreateIndexBuffer(ID3D11Device *pDevice ,int Capablity ,const IndexType* pInitialData ,UINT CPUAccessFlag = 0)
	{
		CD3D11_BUFFER_DESC IndexBufferDesc(sizeof(IndexType)*Capablity, D3D11_BIND_INDEX_BUFFER);
		std::unique_ptr<D3D11_SUBRESOURCE_DATA> pInitialSubresource;
		if (pInitialData){
			pInitialSubresource.reset(new D3D11_SUBRESOURCE_DATA);
			pInitialSubresource->pSysMem = pInitialData;
			pInitialSubresource->SysMemPitch = 0;
			pInitialSubresource->SysMemSlicePitch = 0;
		}
		ID3D11Buffer* pBuffer = nullptr;
		ThrowIfFailed(
			pDevice->CreateBuffer(
				&IndexBufferDesc,
				pInitialSubresource.get(),
				&pBuffer
				)
			);
		return pBuffer;
	}

	inline ID3D11SamplerState* CreateSamplerState(ID3D11Device *pDevice , D3D11_FILTER Filter = D3D11_FILTER_MIN_MAG_MIP_LINEAR , D3D11_TEXTURE_ADDRESS_MODE AddressMode = D3D11_TEXTURE_ADDRESS_WRAP , float MipLODBias = 0.0f ,UINT MaxAnisotropy = 1,D3D11_COMPARISON_FUNC ComparisonFunc = D3D11_COMPARISON_ALWAYS ,DirectX::CXMVECTOR BorderColor = g_XMZero,float MinLOD = 0.0f, float MaxLOD = D3D11_FLOAT32_MAX)
	{
		ID3D11SamplerState *pSamplerState;
		D3D11_SAMPLER_DESC SamplerDesc;
		SamplerDesc.Filter = Filter;
		SamplerDesc.AddressU = AddressMode;
		SamplerDesc.AddressV = AddressMode;
		SamplerDesc.AddressW = AddressMode;
		SamplerDesc.MipLODBias = MipLODBias;
		SamplerDesc.MaxAnisotropy = MaxAnisotropy;
		SamplerDesc.ComparisonFunc = ComparisonFunc;
		XMStoreFloat4((XMFLOAT4*)(&SamplerDesc.BorderColor[0]),BorderColor);
		/*SamplerDesc.BorderColor[0] = 0;
		SamplerDesc.BorderColor[1] = 0;
		SamplerDesc.BorderColor[2] = 0;
		SamplerDesc.BorderColor[3] = 0;*/
		SamplerDesc.MinLOD = MinLOD;
		SamplerDesc.MaxLOD = MaxLOD;

		// Create the texture sampler state.
		ThrowIfFailed(pDevice->CreateSamplerState(&SamplerDesc, &pSamplerState));
		return pSamplerState;
	}

	// Helper sets a D3D resource name string (used by PIX and debug layer leak reporting).
	template<UINT TNameLength>
	inline void SetDebugObjectName(_In_ ID3D11DeviceChild* resource, _In_z_ const char (&name)[TNameLength])
	{
		#if defined(_DEBUG) || defined(PROFILE)
			resource->SetPrivateData(WKPDID_D3DDebugObjectName, TNameLength - 1, name);
		#else
			UNREFERENCED_PARAMETER(resource);
			UNREFERENCED_PARAMETER(name);
		#endif
	}


	// Helper smart-pointers
	struct handle_closer { void operator()(HANDLE h) { if (h) CloseHandle(h); } };

	typedef public std::unique_ptr<void, handle_closer> ScopedHandle;

	inline HANDLE safe_handle( HANDLE h ) { return (h == INVALID_HANDLE_VALUE) ? 0 : h; }


	template<class T> class ScopedObject
	{
	public:
		explicit ScopedObject( T *p = 0 ) : _pointer(p) {}
		~ScopedObject()
		{
			if ( _pointer )
			{
				_pointer->Release();
				_pointer = nullptr;
			}
		}

		bool IsNull() const { return (!_pointer); }

		T& operator*() { return *_pointer; }
		T* operator->() { return _pointer; }
		T** operator&() { return &_pointer; }

		void Reset(T *p = 0) { if ( _pointer ) { _pointer->Release(); } _pointer = p; }

		T* Get() const { return _pointer; }

	private:
		ScopedObject(const ScopedObject&);
		ScopedObject& operator=(const ScopedObject&);
		
		T* _pointer;
	};

}
#endif //__d3d11_1_h__