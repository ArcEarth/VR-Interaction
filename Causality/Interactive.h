#pragma once
#include "Common\DirectXMathExtend.h"
#include <Leap.h>

namespace Platform
{
	enum CursorButtonEnum
	{
		LButton,
		MButton,
		RButton,
		NoButton,
	};

	struct CursorMoveEventArgs
	{
		// Relative position to window's top left corner
		DirectX::Vector2 Position;
		DirectX::Vector2 PositionDelta;
		float WheelDelta;
	};

	class ICursorController abstract
	{
		virtual DirectX::Vector2 CurrentPosition() const = 0;
		virtual DirectX::Vector2 DeltaPosition() const = 0;
		virtual bool IsButtonDown(CursorButtonEnum button) const = 0;
		virtual void SetCursorPosition(const DirectX::Vector2& pos) = 0;
	};

	struct CursorButtonEvent
	{
		CursorButtonEvent() {}
		CursorButtonEvent(const CursorButtonEnum button)
			: Button (button)
		{}
		CursorButtonEnum Button;
	};

	class ICursorInteractive abstract
	{
	public:
		virtual void OnMouseButtonDown(const CursorButtonEvent &e) = 0;
		virtual void OnMouseButtonUp(const CursorButtonEvent &e) = 0;
		virtual void OnMouseMove(const CursorMoveEventArgs &e) = 0;
	};

	enum KeyModifiers
	{
		Mod_Shift = 0x001,
		Mod_Control = 0x002,
		Mod_Meta = 0x004,
		Mod_Alt = 0x008,
	};

	struct KeyboardEventArgs
	{
		unsigned Modifier;
		unsigned Key;
	};

	class IKeybordInteractive abstract
	{
	public:
		virtual void OnKeyDown(const KeyboardEventArgs&e) = 0;
		virtual void OnKeyUp(const KeyboardEventArgs&e) = 0;
	};

	class IJoyStickInteractive abstract
	{
	public:
	};

	struct UserHandsEventArgs
	{
		const Leap::Controller& sender;
	};

	class IUserHandsInteractive abstract
	{
	public:
		virtual void OnHandsTracked(const UserHandsEventArgs& e) = 0;
		virtual void OnHandsTrackLost(const UserHandsEventArgs& e) = 0;
		virtual void OnHandsMove(const UserHandsEventArgs& e) = 0;
	};

	class IUserHeadInteractive abstract
	{
	public:
	};

	class IUserSkeletonInteractive abstract
	{
	public:
	};
}