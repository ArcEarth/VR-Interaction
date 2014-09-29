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
// devicestate.h
//
#pragma once
#include <boost/signals2.hpp>


namespace Audio
{
	//template <class TArg>
	//class EventHandler < TArg >
	//{
	//	void operator(void* sender, TArg*);
	//};

	// NB: All states >= DeviceStateInitialized will allow some methods
	// to be called successfully on the Audio Client
	enum class DeviceState
	{
		DeviceStateUnInitialized,
		DeviceStateInError,
		DeviceStateDiscontinuity,
		DeviceStateFlushing,
		DeviceStateActivated,
		DeviceStateInitialized,
		DeviceStateStarting,
		DeviceStatePlaying,
		DeviceStateCapturing,
		DeviceStatePausing,
		DeviceStatePaused,
		DeviceStateStopping,
		DeviceStateStopped
	};

	// Class for DeviceStateChanged events
	class DeviceStateChangedEventArgs
	{
	public:
		DeviceStateChangedEventArgs(DeviceState newState, HRESULT hr) :
			m_DeviceState(newState),
			m_hr(hr)
		{};

		DeviceState State() const
		{
			return m_DeviceState;
		};

		int hr() const
		{
			return m_hr;
		}

	private:
		DeviceState      m_DeviceState;
		HRESULT          m_hr;
	};

	// DeviceStateChanged delegate
	typedef void(DeviceStateChangedHandlerType)(void *sender,const DeviceStateChangedEventArgs *e);
	typedef std::function<DeviceStateChangedHandlerType> DeviceStateChangedEventHandler;
	// DeviceStateChanged Event
	class AudioDeviceStateChangedEvent
	{
	public:
		AudioDeviceStateChangedEvent() :
			m_DeviceState(DeviceState::DeviceStateUnInitialized)
		{};

		~AudioDeviceStateChangedEvent()
		{
			DeviceStateChanged.disconnect_all_slots();
		}

		DeviceState GetState() const { return m_DeviceState; };

	protected:
		void SetState(DeviceState newState, HRESULT hr, bool FireEvent) {
			if (m_DeviceState != newState)
			{
				m_DeviceState = newState;

				if (FireEvent)
				{
					DeviceStateChangedEventArgs e(m_DeviceState, hr);
					DeviceStateChanged(this, &e);
				}
			}
		};

#pragma warning(push)
#pragma warning(disable: 4512)
	public:
		boost::signals2::signal<DeviceStateChangedHandlerType> DeviceStateChanged;
#pragma warning(pop)
	private:
		DeviceState     m_DeviceState;
	};

}
