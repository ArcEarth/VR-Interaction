//Kinect Driver
#pragma once

#ifndef MYKINECT_H
#define MYKINECT_H


#include "stdafx.h"
#include "resource.h"
#include <Kinect.h>

#include "SpeechCommandStream.h"
#include "KinectFilter.h"

#include <memory>
#include <thread>
#include <wrl\client.h>

class KinectControler
{

public:
	KinectControler();
	~KinectControler();

	IBodyFrame* GetSkeletonFrame();
	SpeechCommands GetSpeechCommand();

	HRESULT StartStream();
	HRESULT StartSkeletonStream();
	HRESULT StartSpeechCommandStream();

protected:

	HRESULT					InitializeNuiSensor();
	HRESULT                 InitializeNuiSensor( OLECHAR * instanceName );
	void                    Release();

private:

	// Current Kinect
	Microsoft::WRL::ComPtr<IKinectSensor>          m_pKinectSensor;
	Microsoft::WRL::ComPtr<ICoordinateMapper>      m_pCoordinateMapper;
	// Body reader
	Microsoft::WRL::ComPtr<IBodyFrameReader>       m_pBodyFrameReader;

	bool									m_ThreadTerminatingSignal;
	std::unique_ptr<std::thread>			m_pStreamingThread;

	std::unique_ptr<SpeechCommandStream>	m_pCommandStream;

	static LPCWSTR							GrammarFileName;

};


#endif //MYKINECT_H