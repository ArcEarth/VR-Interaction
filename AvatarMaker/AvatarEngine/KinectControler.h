//Kinect Driver
#pragma once

#ifndef MYKINECT_H
#define MYKINECT_H


#include "stdafx.h"
#include "resource.h"
#include <NuiApi.h>
#include "SpeechCommandStream.h"
#include "KinectFilter.h"
#include "NuiSkeletonStream.h"
#include "SkeletonStreamBufferViewer.h"
//#include "SkeletonStreamViewer.h"
#include <memory>
#include <thread>

class KinectControler
{

public:
	KinectControler();
	~KinectControler();

	NUI_SKELETON_FRAME* GetSkeletonFrame();
	SpeechCommands GetSpeechCommand();

	HRESULT StartStream();
	HRESULT StartSkeletonStream();
	HRESULT StartSpeechCommandStream();

protected:

	HRESULT					InitializeNuiSensor();
	HRESULT                 InitializeNuiSensor( OLECHAR * instanceName );
	void                    Release();

private:
	// Current kinect
	INuiSensor*								m_pNuiSensor;
	
	std::unique_ptr<SkeletonStreamBufferViewer> m_pSkeletonStreamBuffer;
	std::unique_ptr<Nui::NuiSkeletonStream> m_pSkeletonStream;
	bool									m_ThreadTerminatingSignal;
	std::unique_ptr<std::thread>			m_pStreamingThread;

	std::unique_ptr<SpeechCommandStream>	m_pCommandStream;

	static LPCWSTR							GrammarFileName;

};


#endif //MYKINECT_H