//*********************************************************
//
// Copyright (c) Microsoft. All rights reserved.
// THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
// IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
// PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
//
//*********************************************************

//
// pch.h
// Header for standard system include files.
//

#pragma once
#include <wrl\implements.h>
#include <wrl.h>
#include <mmreg.h>
#include <mfapi.h>
#include "..\Common.hpp"


namespace Platform
{

	template <class T> void SafeRelease(__deref_inout_opt T **ppT)
	{
		T *pTTemp = *ppT;    // temp copy
		*ppT = nullptr;      // zero the input
		if (pTTemp)
		{
			pTTemp->Release();
		}
	}

	// Exception Helper Method
	inline void ThrowIfFailed(HRESULT hr)
	{
		if (FAILED(hr))
		{
			// Set a breakpoint on this line to catch API errors.
			throw;
		}
	}

	namespace Audio
	{

		template <class T_Parent, HRESULT(T_Parent::*CallbackFunc)(IMFAsyncResult*)>
		class MFAsyncCallback : public IMFAsyncCallback
		{
		public:
			//T_Parent* parent , HRESULT(T_Parent::*pCallback)(IMFAsyncResult*)
			MFAsyncCallback(T_Parent* parent)
				: _parent(parent)
				, _dwQueueID(MFASYNC_CALLBACK_QUEUE_MULTITHREADED)
			{
			}
			STDMETHOD_(ULONG, AddRef)()
			{
				return _parent->AddRef();
			}
			STDMETHOD_(ULONG, Release)()
			{
				return _parent->Release();
			}
			STDMETHOD(QueryInterface)(REFIID riid, void **ppvObject)
			{
				if (riid == IID_IMFAsyncCallback || riid == IID_IUnknown)
				{
					(*ppvObject) = this;
					AddRef();
					return S_OK;
				}
				*ppvObject = NULL;
				return E_NOINTERFACE;
			}

			STDMETHOD(GetParameters)(
				/* [out] */ __RPC__out DWORD *pdwFlags,
				/* [out] */ __RPC__out DWORD *pdwQueue)
			{
				*pdwFlags = 0;
				*pdwQueue = _dwQueueID;
				return S_OK;
			}

			STDMETHOD(Invoke)( /* [out] */ __RPC__out IMFAsyncResult * pResult)
			{
				(_parent->*CallbackFunc)(pResult);
				return S_OK;
			}

			void SetQueueID(DWORD dwQueueID) { _dwQueueID = dwQueueID; }

		protected:
			T_Parent* _parent;
			//HRESULT(T_Parent::*CallbackFunc)(IMFAsyncResult*);
			DWORD   _dwQueueID;
		};

		//
		// CAsyncState
		//
		// Used to maintain state during MF Work Item callbacks
		class CAsyncState :
			public Microsoft::WRL::RuntimeClass < Microsoft::WRL::RuntimeClassFlags< Microsoft::WRL::ClassicCom>, IUnknown >
		{
		public:
			CAsyncState(const std::vector<int> *pData, UINT32 size) :
				Data(pData),
				Size(size)
			{
			};
			virtual ~CAsyncState() {};
		public:
			const std::vector<int>*		Data;
			UINT32						Size;

		private:
		};

		struct RenderBuffer
		{
			UINT32          BufferSize;
			UINT32          BytesFilled;
			BYTE           *Buffer;
			RenderBuffer   *Next;

			RenderBuffer() :
				BufferSize(0),
				BytesFilled(0),
				Buffer(nullptr),
				Next(nullptr)
			{
			}

			~RenderBuffer()
			{
				if (Buffer) delete []Buffer;
			}
		};

		enum RenderSampleType
		{
			SampleTypeUnknown,
			SampleTypeFloat,
			SampleType16BitPCM,
		};

		inline RenderSampleType CalculateMixFormatType(const WAVEFORMATEX *wfx)
		{
			if ((wfx->wFormatTag == WAVE_FORMAT_PCM) ||
				((wfx->wFormatTag == WAVE_FORMAT_EXTENSIBLE) &&
				(reinterpret_cast<const WAVEFORMATEXTENSIBLE *>(wfx)->SubFormat == KSDATAFORMAT_SUBTYPE_PCM)))
			{
				if (wfx->wBitsPerSample == 16)
				{
					return RenderSampleType::SampleType16BitPCM;
				}
			}
			else if ((wfx->wFormatTag == WAVE_FORMAT_IEEE_FLOAT) ||
				((wfx->wFormatTag == WAVE_FORMAT_EXTENSIBLE) &&
				(reinterpret_cast<const WAVEFORMATEXTENSIBLE *>(wfx)->SubFormat == KSDATAFORMAT_SUBTYPE_IEEE_FLOAT)))
			{
				return RenderSampleType::SampleTypeFloat;
			}

			return RenderSampleType::SampleTypeUnknown;
		}
	}
}
