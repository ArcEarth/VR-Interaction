#pragma once
#include "nuistreamviewer.h"
#include <deque>
#include <mutex>
class SkeletonStreamBufferViewer :
	public Nui::INuiStreamViewer
{
public:
	SkeletonStreamBufferViewer(void)
		: m_paused(false)
	{
	}

	SkeletonStreamBufferViewer(size_t BufferSize)
		: m_StreamingBuffer(BufferSize)
		, m_paused(false)
	{
	}

	~SkeletonStreamBufferViewer(void)
	{
		m_paused = true;
	}

	virtual void SetSkeleton(const NUI_SKELETON_FRAME* pFrame)
	{
		if (!pFrame || m_paused) return;

		m_BufferMutex.lock();
		if (m_StreamingBuffer.size() >= m_StreamingBuffer.max_size()) 
			m_StreamingBuffer.pop_front();
		m_StreamingBuffer.emplace_back(*pFrame);
		m_BufferMutex.unlock();
	}

	NUI_SKELETON_FRAME* GetSkeleton()
	{
		if (m_StreamingBuffer.empty()) return nullptr;

		m_BufferMutex.lock();
		m_ReadingBuffer = m_StreamingBuffer.front();
		m_StreamingBuffer.pop_front();
		m_BufferMutex.unlock();

		return &m_ReadingBuffer;
	}

	NUI_SKELETON_FRAME* GetLatestSkeleton()
	{
		if (m_StreamingBuffer.empty()) return nullptr;

		m_BufferMutex.lock();
		m_ReadingBuffer = m_StreamingBuffer.back();
		m_StreamingBuffer.clear();
		m_BufferMutex.unlock();

		return &m_ReadingBuffer;
	}

	bool empty() const 
	{
		return m_StreamingBuffer.empty();
	}

	virtual void PauseSkeleton(bool pause)
	{
		m_paused = pause;
	}

	virtual void SetImage(const Nui::NuiImageBuffer* pImage)
	{
	}

	virtual void SetImageType(NUI_IMAGE_TYPE type) 
	{
	}

private:
	bool m_paused;
	NUI_SKELETON_FRAME				m_ReadingBuffer;
	std::deque<NUI_SKELETON_FRAME>	m_StreamingBuffer;
	std::mutex						m_BufferMutex;
};

