#pragma once
#include "BCL.h"
#include "Armature.h"

namespace Causality
{
	class FbxModel
	{
	public:
		bool LoadFromFile(const string& file);
		const AnimationSpace& GetAnimationSpace() const;

	private:
		class Impl;
		unique_ptr<Impl> m_pImpl;
	};
}