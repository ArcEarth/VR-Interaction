//------------------------------------------------------------------------------
// <copyright file="INuiStreamViewer.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#pragma once

#include "NuiUtility.h"
#include <NuiApi.h>
#include "NuiImageBuffer.h"

namespace Nui
{
	//class INuiSkeletonStreamViewer
	//{
	//public:
	//	//	/// <summary>
	//	//	/// Set the buffer containing the image pixels.
	//	//	/// </summary>
	//	//	/// <param name="pImage">The pointer to image buffer object</param>
	//	virtual void SetSkeleton(const NUI_SKELETON_FRAME* pFrame) = 0;
	//	/// <summary>
	//	/// Pause the skeleton
	//	/// </summary>
	//	/// <param name="pause">Pause or resume the skeleton</param>
	//	virtual void PauseSkeleton(bool pause) = 0;
	//};

	//class INuiImageStreamViewer
	//{
	//public:
	//	/// <summary>
	//	/// Set the buffer containing the image pixels.
	//	/// </summary>
	//	/// <param name="pImage">The pointer to image buffer object</param>
	//	virtual void SetImage(const NuiImageBuffer* pImage) = 0;

	//	/// <summary>
	//	/// Set image type.
	//	/// </summary>
	//	/// <param name="type">Image type to be set</param>
	//	virtual void SetImageType(NUI_IMAGE_TYPE type) = 0;
	//};

	class INuiStreamViewer
	{
	public:
		virtual ~INuiStreamViewer(){}
		/// <summary>
		/// Set the buffer containing the image pixels.
		/// </summary>
		/// <param name="pImage">The pointer to image buffer object</param>
		virtual void SetImage(const NuiImageBuffer* pImage) = 0;

		/// <summary>
		/// Set image type.
		/// </summary>
		/// <param name="type">Image type to be set</param>
		virtual void SetImageType(NUI_IMAGE_TYPE type) = 0;
		//	/// <summary>
		//	/// Set the buffer containing the image pixels.
		//	/// </summary>
		//	/// <param name="pImage">The pointer to image buffer object</param>
		virtual void SetSkeleton(const NUI_SKELETON_FRAME* pFrame) = 0;
		/// <summary>
		/// Pause the skeleton
		/// </summary>
		/// <param name="pause">Pause or resume the skeleton</param>
		virtual void PauseSkeleton(bool pause) = 0;
	}; 
}