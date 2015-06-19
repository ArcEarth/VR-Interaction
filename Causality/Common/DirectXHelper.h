﻿#pragma once
#define NOMINMAX
#include <ppltasks.h>	// For create_task
#include <fstream>
#include <filesystem>
#include <VertexTypes.h>

#if defined(_XBOX_ONE) && defined(_TITLE)
#include <d3d11_x.h>
#define NO_D3D11_DEBUG_NAME
#else
#include <d3d11_2.h>
#endif

#if !defined(NO_D3D11_DEBUG_NAME) && ( defined(_DEBUG) || defined(PROFILE) )
#pragma comment(lib,"dxguid.lib")
#endif

//#include <exception>

#pragma warning(push)
#pragma warning(disable : 4005)
#include <stdint.h>
#pragma warning(pop)

#include <DirectXMath.h>

#ifndef HAS_MEMBER_FUNCTION
#define HAS_MEMBER_FUNCTION(member_name, name)                                   \
    template<typename T, typename Signature>                            \
    struct name {                                                       \
        typedef char yes[1];                                            \
        typedef char no [2];                                            \
        template <typename U, U> struct type_check;                     \
        template <typename _1> static yes &chk(type_check<Signature, &_1::member_name > *); \
        template <typename   > static no  &chk(...);                    \
        static bool const value = sizeof(chk<T>(0)) == sizeof(yes);     \
    }
#endif // !HAS_MEM_FUNC

#ifndef HAS_MEMBER
#define HAS_MEMBER(member_name, name)                                   \
    template<typename T>												\
    struct name {                                                       \
        typedef char yes[1];                                            \
        typedef char no [2];                                            \
        template <typename> static yes &chk(decltype(T::member_name)*);			\
        template <typename> static no  &chk(...);                    \
        static bool const value = sizeof(chk<T>(0)) == sizeof(yes);     \
    }
#endif // !HAS_MEM_FUNC




namespace DirectX
{
	using Microsoft::WRL::ComPtr;

	namespace VertexTraits
	{
		HAS_MEMBER(position, has_position);
		HAS_MEMBER(normal, has_normal);
		HAS_MEMBER(textureCoordinate, has_tex);
		HAS_MEMBER(color, has_color);
		HAS_MEMBER(tanget, has_tanget);
		HAS_MEMBER(weights, has_weights);

	}

#if defined (__cplusplus_winrt)
	inline Platform::String^ ErrorDescription(HRESULT hr)
	{
		if (FACILITY_WINDOWS == HRESULT_FACILITY(hr))
			hr = HRESULT_CODE(hr);
		TCHAR* szErrMsg;

		auto size = FormatMessage(
			FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM,
			NULL, hr, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
			(LPTSTR) &szErrMsg, 0, NULL);

		if (!size) {
			Platform::String^ msg = ref new Platform::String(szErrMsg);
			LocalFree(szErrMsg);
			return msg;
		}
		else
		{
			return "Undifined Error Code";
		}
	}
	inline void ThrowIfFailed(HRESULT hr)
	{
		if (FAILED(hr))
		{
			auto ErrMsg = ErrorDescription(hr);
			throw Platform::Exception::CreateException(hr);
		}
	}
#else
	inline TCHAR* ErrorDescription(HRESULT hr)
	{
		if (FACILITY_WINDOWS == HRESULT_FACILITY(hr))
			hr = HRESULT_CODE(hr);
		static TCHAR szErrMsg[256];
		static TCHAR dfErrMsg[] = L"Unrecongnized Error Code";
		auto size = FormatMessage(
			FORMAT_MESSAGE_FROM_SYSTEM,
			NULL, hr, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
			szErrMsg, ARRAYSIZE(szErrMsg), NULL);

		if (!size) {
			//std::string msg(szErrMsg);
			//LocalFree(szErrMsg);
			return szErrMsg;
		}
		else
		{
			return dfErrMsg;
		}
	}
	inline void ThrowIfFailed(HRESULT hr)
	{
		if (FAILED(hr))
		{
			auto ErrMsg = ErrorDescription(hr);
			std::wstring_convert<std::codecvt_utf8<TCHAR>, TCHAR> utfconvt;
			auto str = utfconvt.to_bytes(ErrMsg);
			throw std::exception(str.c_str());
		}
	}
#endif






	// simliar to std::lock_guard for exception-safe Direct3D 11 resource locking
	class MapGuard : public D3D11_MAPPED_SUBRESOURCE
	{
	public:
		MapGuard(_In_ ID3D11DeviceContext* context,
			_In_ ID3D11Resource *resource,
			_In_ UINT subresource,
			_In_ D3D11_MAP mapType,
			_In_ UINT mapFlags)
			: mContext(context), mResource(resource), mSubresource(subresource)
		{
			ThrowIfFailed(
				mContext->Map(resource, subresource, mapType, mapFlags, this));
		}

		~MapGuard()
		{
			mContext->Unmap(mResource, mSubresource);
		}

		uint8_t* get() const
		{
			return reinterpret_cast<uint8_t*>(pData);
		}
		uint8_t* get(size_t slice) const
		{
			return reinterpret_cast<uint8_t*>(pData) + (slice * DepthPitch);
		}

		uint8_t* scanline(size_t row) const
		{
			return reinterpret_cast<uint8_t*>(pData) + (row * RowPitch);
		}
		uint8_t* scanline(size_t slice, size_t row) const
		{
			return reinterpret_cast<uint8_t*>(pData) + (slice * DepthPitch) + (row * RowPitch);
		}

	private:
		ID3D11DeviceContext*    mContext;
		ID3D11Resource*         mResource;
		UINT                    mSubresource;

		MapGuard(MapGuard const&);
		MapGuard& operator= (MapGuard const&);
	};


	// Helper sets a D3D resource name string (used by PIX and debug layer leak reporting).
	template<UINT TNameLength>
	inline void SetDebugObjectName(_In_ ID3D11DeviceChild* resource, _In_z_ const char(&name)[TNameLength])
	{
#if !defined(NO_D3D11_DEBUG_NAME) && ( defined(_DEBUG) || defined(PROFILE) )
		resource->SetPrivateData(WKPDID_D3DDebugObjectName, TNameLength - 1, name);
#else
		UNREFERENCED_PARAMETER(resource);
		UNREFERENCED_PARAMETER(name);
#endif
	}


	//inline void ThrowIfFailed(HRESULT hr)
	//{
	//	if (FAILED(hr))
	//	{
	//		// Set a breakpoint on this line to catch Win32 API errors.
	//		throw Platform::Exception::CreateException(hr);
	//	}
	//}

	// Function that reads from a binary file asynchronously.
	inline Concurrency::task<std::vector<byte>> ReadDataAsync(const std::wstring& filename)
	{
		Concurrency::task<std::vector<byte>> RetriveDateTask([filename]()->std::vector < byte > {
			std::ifstream fin;
			//auto current = std::tr2::sys::current_path<std::tr2::sys::wpath>();
			//auto dir = current.directory_string();
			fin.open(filename, std::ios::in | std::ios::binary);
			if (fin.is_open())
			{
				return std::vector<byte>((std::istreambuf_iterator<char>(fin)), std::istreambuf_iterator<char>());
			}
			else
			{
				throw std::exception("File not found exception.");
			}
		});
		return RetriveDateTask;
		//using namespace Windows::Storage;
		//using namespace Concurrency;

		//auto folder = Windows::ApplicationModel::Package::Current->InstalledLocation;

		//return create_task(folder->GetFileAsync(Platform::StringReference(filename.c_str()))).then([] (StorageFile^ file) 
		//{
		//	return FileIO::ReadBufferAsync(file);
		//}).then([] (Streams::IBuffer^ fileBuffer) -> std::vector<byte> 
		//{
		//	std::vector<byte> returnBuffer;
		//	returnBuffer.resize(fileBuffer->Length);
		//	Streams::DataReader::FromBuffer(fileBuffer)->ReadBytes(Platform::ArrayReference<byte>(returnBuffer.data(), fileBuffer->Length));
		//	return returnBuffer;
		//});
	}

	// Converts a length in device-independent pixels (DIPs) to a length in physical pixels.
	inline float ConvertDipsToPixels(float dips, float dpi)
	{
		static const float dipsPerInch = 96.0f;
		return floorf(dips * dpi / dipsPerInch + 0.5f); // Round to nearest integer.
	}

#if defined(_DEBUG)
	// Check for SDK Layer support.
	inline bool SdkLayersAvailable()
	{
		HRESULT hr = D3D11CreateDevice(
			nullptr,
			D3D_DRIVER_TYPE_NULL,       // There is no need to create a real hardware device.
			0,
			D3D11_CREATE_DEVICE_DEBUG,  // Check for the SDK layers.
			nullptr,                    // Any feature level will do.
			0,
			D3D11_SDK_VERSION,          // Always set this to D3D11_SDK_VERSION for Windows Store apps.
			nullptr,                    // No need to keep the D3D device reference.
			nullptr,                    // No need to know the feature level.
			nullptr                     // No need to keep the D3D device context reference.
			);

		return SUCCEEDED(hr);
	}
#endif

	template <typename ConstantType>
	inline ID3D11Buffer* CreateConstantBuffer(ID3D11Device *pDevice)
	{
		D3D11_BUFFER_DESC desc = { 0 };

		desc.ByteWidth = sizeof(ConstantType);
		desc.Usage = D3D11_USAGE_DYNAMIC;
		desc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
		desc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
		ID3D11Buffer* pBuffer = nullptr;
		ThrowIfFailed(
			pDevice->CreateBuffer(&desc, nullptr, &pBuffer)
			);

		return pBuffer;
	}

	template <typename T>
	void SetBufferData(ID3D11Buffer *pBuffer, _In_ ID3D11DeviceContext* pContext, T const& value)
	{
		D3D11_MAPPED_SUBRESOURCE mappedResource;

		ThrowIfFailed(
			pContext->Map(pBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &mappedResource)
			);

		*(T*) mappedResource.pData = value;

		pContext->Unmap(pBuffer, 0);
	}


	template <class VertexType>
	inline ComPtr<ID3D11Buffer> CreateVertexBuffer(ID3D11Device *pDevice, int Capablity, const VertexType* pInitialData, UINT CPUAccessFlag = 0)
	{
		ComPtr<ID3D11Buffer> pBuffer(nullptr);
		CD3D11_BUFFER_DESC VertexBufferDesc(sizeof(VertexType)*Capablity, D3D11_BIND_VERTEX_BUFFER, D3D11_USAGE_DEFAULT, CPUAccessFlag);
		if (pInitialData) {
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
		}
		else {
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


	typedef std::shared_ptr<std::vector<D3D11_INPUT_ELEMENT_DESC>> InputDescription;

	inline ComPtr<ID3D11InputLayout> CreateInputLayout(ID3D11Device *pDevice,const InputDescription& pInputDescription, const void* vertexShaderBytecode, size_t bytecodeLength)
	{
		ComPtr<ID3D11InputLayout> pLayout;
		DirectX::ThrowIfFailed(
			pDevice->CreateInputLayout(
			&pInputDescription->at(0),
			pInputDescription->size(),
			vertexShaderBytecode,
			bytecodeLength,
			&pLayout));
		return pLayout;
	}

	template <class VertexType>
	inline ComPtr<ID3D11InputLayout> CreateInputLayout(ID3D11Device *pDevice, const void* vertexShaderBytecode, size_t bytecodeLength)
	{
		static_assert(VertexType::InputElementCount, "Valiad Vertex Type should have static InputElements/InputElementCount member");
		ComPtr<ID3D11InputLayout> pLayout;
		DirectX::ThrowIfFailed(
			pDevice->CreateInputLayout(
			VertexType::InputElements,
			VertexType::InputElementCount,
			vertexShaderBytecode,
			bytecodeLength,
			&pLayout));
		return pLayout;
	}

	template <class IndexType>
	inline ComPtr<ID3D11Buffer> CreateIndexBuffer(ID3D11Device *pDevice, int Capablity, const IndexType* pInitialData, UINT CPUAccessFlag = 0)
	{
		ComPtr<ID3D11Buffer> pBuffer(nullptr);
		CD3D11_BUFFER_DESC IndexBufferDesc(sizeof(IndexType)*Capablity, D3D11_BIND_INDEX_BUFFER);
		std::unique_ptr<D3D11_SUBRESOURCE_DATA> pInitialSubresource;
		if (pInitialData) {
			pInitialSubresource.reset(new D3D11_SUBRESOURCE_DATA);
			pInitialSubresource->pSysMem = pInitialData;
			pInitialSubresource->SysMemPitch = 0;
			pInitialSubresource->SysMemSlicePitch = 0;
		}
		ThrowIfFailed(
			pDevice->CreateBuffer(
			&IndexBufferDesc,
			pInitialSubresource.get(),
			&pBuffer
			)
			);
		return pBuffer;
	}

	inline ID3D11SamplerState* CreateSamplerState(ID3D11Device *pDevice, D3D11_FILTER Filter = D3D11_FILTER_MIN_MAG_MIP_LINEAR, D3D11_TEXTURE_ADDRESS_MODE AddressMode = D3D11_TEXTURE_ADDRESS_WRAP, float MipLODBias = 0.0f, UINT MaxAnisotropy = 1, D3D11_COMPARISON_FUNC ComparisonFunc = D3D11_COMPARISON_ALWAYS, DirectX::CXMVECTOR BorderColor = g_XMZero, float MinLOD = 0.0f, float MaxLOD = D3D11_FLOAT32_MAX)
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
		XMStoreFloat4((XMFLOAT4*) (&SamplerDesc.BorderColor[0]), BorderColor);
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

	// Helper smart-pointers
	struct handle_closer { void operator()(HANDLE h) { if (h) CloseHandle(h); } };

	typedef public std::unique_ptr<void, handle_closer> ScopedHandle;

	inline HANDLE safe_handle(HANDLE h) { return (h == INVALID_HANDLE_VALUE) ? 0 : h; }


	template<class T> class ScopedObject
	{
	public:
		explicit ScopedObject(T *p = 0) : _pointer(p) {}
		~ScopedObject()
		{
			if (_pointer)
			{
				_pointer->Release();
				_pointer = nullptr;
			}
		}

		bool IsNull() const { return (!_pointer); }

		T& operator*() { return *_pointer; }
		T* operator->() { return _pointer; }
		T** operator&() { return &_pointer; }

		void Reset(T *p = 0) { if (_pointer) { _pointer->Release(); } _pointer = p; }

		T* Get() const { return _pointer; }

	private:
		ScopedObject(const ScopedObject&);
		ScopedObject& operator=(const ScopedObject&);

		T* _pointer;
	};
}
