#pragma once
#include "BCL.h"
#include "Armature.h"

namespace Causality
{
	class FbxAnimationParser
	{
	public:
		unique_ptr<BehavierSpace> LoadFromFile(const string& file);
		~FbxAnimationParser();
		FbxAnimationParser();
	private:
		struct Impl;
		unique_ptr<Impl> m_pImpl;
	};
}